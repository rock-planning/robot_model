#include "robot_model/RobotModel.hpp"
#include <omp.h>
#include <iostream>
#include <boost/iterator/iterator_concepts.hpp>

namespace robot_model
{

/////////////////////////////////////////////////out of class  function for comparing link name//////////////////////////////
std::string _removeLinkName;

bool isLinkListed(const urdf::LinkSharedPtr &remove_link)
{
    return (remove_link->name == _removeLinkName);
}

/////////////////////////////////////////////////End of out of class members and variables////////////////////////////////////

RobotModel::RobotModel(RobotModelConfig robot_model_config, double link_padding  )
{
    urdf_file_abs_path_  = robot_model_config.urdf_file;
    srdf_file_abs_path_  = robot_model_config.srdf_file;
    planning_group_name_ = robot_model_config.planning_group_name;
    link_padding_        = link_padding;
}

void RobotModel::setWorldCollisionDetector(collision_detection::AbstractCollisionPtr collision_detector)
{
    world_collision_detector_ = collision_detector;    
    robot_collision_detector_->assignWorldDetector(world_collision_detector_);    
}

bool RobotModel::initialiseURDFandSRDF()
{
    std::string xml_string;
    std::fstream xml_file(urdf_file_abs_path_.c_str(), std::fstream::in);
    bool srdf_ok_ = false ;

    srdf_model_.reset(new srdf::Model());
    if (xml_file.is_open())
    {
        while ( xml_file.good() )
        {
            std::string line;
            std::getline( xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        urdf_model_ = urdf::parseURDF(xml_string);
        if(urdf_model_.get() == NULL)
        {
            LOG_ERROR("[RobotModel] Error while getting urdf model. urdf_model is empty");
            return false;
        }
        world_frame_ = urdf_model_->getRoot()->name;
    }
    else
    {
        LOG_ERROR("[RobotModel] Cannot open urdf file.");
        return false;
    }

    srdf_ok_ = srdf_model_->initFile(*urdf_model_, srdf_file_abs_path_);

    if(!srdf_ok_)
    {
        LOG_ERROR("[RobotModel] Error while initialising srdf model");
        return srdf_ok_;
    }

    if (!kdl_parser::treeFromFile(urdf_file_abs_path_, kdl_tree_))
    {
        LOG_ERROR("[RobotModel] Error while initialising kdl treey");
        return false;
    }
    return true;
}

bool RobotModel::initialization()
{
    // initialse urdf and srdf
    bool res = initialiseURDFandSRDF();

    if(!res)
    {
        LOG_FATAL("[RobotModel] Robot model initialisation failed");
        return false;
    }

    // Initialise the robot joints


    for( std::map<std::string, urdf::JointSharedPtr >::iterator it = urdf_model_->joints_.begin(); it != urdf_model_->joints_.end(); it++)
    {
        LOG_DEBUG("[RobotModel] Visiting the joint:%s and it is of type %d", it->first.c_str(), it->second->type );

        if( (it->second->type != urdf::Joint::FIXED) && (it->second->type != urdf::Joint::UNKNOWN) && (it->second->type != urdf::Joint::CONTINUOUS) )
        {
            double joint_value;
            std::string joint_name;
            RobotJoint robot_joint;

            if( (it->second->limits->lower > 0) || (it->second->limits->upper < 0) )            
                joint_value = (it->second->limits->lower + it->second->limits->upper )/2.0;            
            else
                joint_value = 0;

            joint_name = it->first;
            robot_joint.setJointValue(joint_value);
            robot_joint.setJointName(joint_name);
            robot_joint.setJointInfo(*(it->second.get()));
            // incase of mimic joints
            if (it->second->mimic != nullptr)
            {
                robot_joint.setJointAsMimic();
                robot_joint.mimic_joints_ = MimicJoint( it->second->mimic->joint_name,
                                                        it->second->mimic->multiplier, it->second->mimic->offset );
            }
            // Assign robot joint to the robot state
            robot_state_.robot_joints_[joint_name] = robot_joint;
        }
    }

    for (std::map<std::string, RobotJoint>::iterator it = robot_state_.robot_joints_.begin(); it!= robot_state_.robot_joints_.end(); ++it)
    {
        if(it->second.isMimicJoint())
        {
            if( robot_state_.robot_joints_.find( it->second.mimic_joints_.joint_to_mimic ) == robot_state_.robot_joints_.end() )
            {
                LOG_DEBUG("[RobotModel]: The mimic joint name: %s is not available", it->second.mimic_joints_.joint_to_mimic.c_str());
                return false;
            }
            else
            {
                robot_state_.robot_joints_[it->second.mimic_joints_.joint_to_mimic].mimic_joints_map_[it->first]  = robot_state_.robot_joints_[it->first].mimic_joints_;
                LOG_DEBUG("[RobotModel]: The joint %s is mimicing the joint %s \n", it->second.mimic_joints_.joint_to_mimic.c_str(), it->first.c_str());
            }
        }
    }

    // Initialise robot link
    std::string link_name;    
    RobotLink robot_link;    

    auto start_initialisation = std::chrono::high_resolution_clock::now();    

    for(std::map<std::string, urdf::LinkSharedPtr >::iterator it = urdf_model_->links_.begin(); it != urdf_model_->links_.end(); it++)
    {
        link_name = it->second->name;
        robot_link.setLinkDFSVisited(false);
        robot_link.setLinkName(link_name);
        robot_state_.robot_links_[link_name] = robot_link;

        //get visual and collision data from urdf and assign to the robot state
        std::vector<urdf::VisualSharedPtr > visual_array        = urdf_model_->getLink(link_name)->visual_array;
        std::vector<urdf::CollisionSharedPtr > collision_array  = urdf_model_->getLink(link_name)->collision_array;
        robot_state_.robot_links_[link_name].setLinkVisuals( visual_array );
        robot_state_.robot_links_[link_name].setLinkCollisions(collision_array );

        
        // Reason for removing these function:
        // ===================================
        // Saving the link as pointcloud is deprecated as self filtering the robot link
        // using the pointcloud seems to be computation expensive. Creating a plc::concavehull did
        // gave a good result at the cost of heavy computation and self-filtering using
        // pcl::convexhull remove more pointcloud than it suppose to remove. !
        
        // convert visual and collision data to pointcloud and then assing to the robot state
        //std::vector<pcl::PointCloud<pcl::PointXYZ> > link_visual_point_cloud, link_collision_point_cloud;        
        // create pointcloud from visual link        
        //registerLinks(visual_array, link_visual_point_cloud);
        // create pointcloud from collision link
        //registerLinks(collision_array, link_collision_point_cloud);
        //robot_state_.robot_links_[link_name].setVisualPointCloud(link_visual_point_cloud);
        //robot_state_.robot_links_[link_name].setCollisionPointCloud(link_collision_point_cloud);
    }

    std::vector<std::string>  list_of_visited_link_from_root;
    dfsTraversing(urdf_model_->getRoot()->name, list_of_visited_link_from_root);
    settingVisitedFlagLinkToFalse();
    if(!initializeLinksCollisions())
        return false;

    auto finish_initialisation = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> elapsed = std::chrono::duration_cast<std::chrono::duration<double>> (finish_initialisation - start_initialisation);    

    LOG_INFO("[RobotModel] Robot link initialisation took %f seconds", elapsed.count());
    return true;
}

void RobotModel::dfsTraversing(std::string start_link_name, std::vector<std::string>& visited_links)
{
    robot_state_.robot_links_[start_link_name].setLinkDFSVisited(true);
    visited_links.push_back(start_link_name);

    //std::string child_link_name;
    int joint_type;
    std::string joint_name;
    bool is_link_visited;
    KDL::JntArray kdl_chain_joint_array;
    KDL::Frame Child_link_Frame_in_Parent_Frame;
    KDL::Frame Parent_link_in_base_link;
    KDL::Frame Child_link_Frame_in_base_link;

    std::vector<urdf::LinkSharedPtr > child_links = urdf_model_->getLink(start_link_name)->child_links;

    //for(std::size_t i = 0; i < child_links.size(); i++)
    //for(std::vector<urdf::LinkSharedPtr >::iterator it = child_links.begin(); it != child_links.end(); it++)
    for(auto &cl : child_links)    
    {
        //child_link_name     = child_links.at(i)->name;
        //child_link_name     = cl->name;
        is_link_visited     = robot_state_.robot_links_[cl->name].getLinkDFSVisited();
        urdf::JointSharedPtr joint_between_child_link_and_parent_link = urdf_model_->getLink(cl->name)->parent_joint;
        joint_name          = joint_between_child_link_and_parent_link->name;
        joint_type          = joint_between_child_link_and_parent_link->type;

        if(!is_link_visited)
        {
            kdl_tree_.getChain(start_link_name, cl->name, kdl_chain_);
            KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
            kdl_chain_joint_array.resize(kdl_chain_.getNrOfJoints());
            if(joint_type != urdf::Joint::FIXED)
            {
                kdl_chain_joint_array.data[0]=robot_state_.robot_joints_[joint_name].getJointValue();
                 LOG_DEBUG( "[RobotModel] DFSTraversing : The joint between %s and %s is of type %f and is of value %d", 
                             start_link_name.c_str(), cl->name.c_str(), joint_type, kdl_chain_joint_array.data[0] ); 
          }

          fk_solver.JntToCart(kdl_chain_joint_array, Child_link_Frame_in_Parent_Frame);

          // if the parent link is root no more composing frames, the link pose has been already calculated 
          if(start_link_name == urdf_model_->getRoot()->name)
          {
              robot_state_.robot_links_[cl->name].setLinkFrame(Child_link_Frame_in_Parent_Frame);

              //LOG_DEBUG("============================================================================== ");
              //LOG_DEBUG("Frame: %s", child_link_name.c_str());
              //LOG_DEBUG("x :%f", Child_link_Frame_in_Parent_Frame.p.x());
              //LOG_DEBUG("y :%f", Child_link_Frame_in_Parent_Frame.p.y());
              //LOG_DEBUG("z :%f ", Child_link_Frame_in_Parent_Frame.p.z());
              //LOG_DEBUG("============================================================================== ");
          }
          else
          {
              Parent_link_in_base_link        = robot_state_.robot_links_[start_link_name].getLinkFrame();
              Child_link_Frame_in_base_link   = Parent_link_in_base_link * Child_link_Frame_in_Parent_Frame;
              robot_state_.robot_links_[cl->name].setLinkFrame(Child_link_Frame_in_base_link);

              //LOG_DEBUG("============================================================================== ");
              //LOG_DEBUG("Frame: %s w.r.t frame: %s", child_link_name.c_str(), start_link_name.c_str());
              //LOG_DEBUG("x :%f", Child_link_Frame_in_base_link.p.x());
              //LOG_DEBUG("y :%f", Child_link_Frame_in_base_link.p.y());
              //LOG_DEBUG("z :%f", Child_link_Frame_in_base_link.p.z());
              //LOG_DEBUG("============================================================================== ");

            }
            // now we have to calculate the position of the visuals and collision of the robot in the world coordinate (global pose)
            //robot_state_.robot_links_[cl->name].calculateLinkVisualsPoseInGlobalPose();
            robot_state_.robot_links_[cl->name].calculateLinkCollisionPoseinGlobalPose();

            // recursive dfs 
            dfsTraversing(cl->name, visited_links);

        }
    }
}

void RobotModel::settingVisitedFlagLinkToFalse()
{
    for( std::map<std::string, RobotLink>::iterator it = robot_state_.robot_links_.begin(); it != robot_state_.robot_links_.end(); it++ )
    {    
        it->second.setLinkDFSVisited(false);
    }
}

void RobotModel::settingVisitedFlagLinkToFalse(std::vector<std::string> visted_links_names)
{
    std::string link_name;
    for(std::size_t i=0; i < visted_links_names.size(); i++)
    {
        link_name = visted_links_names.at(i);
        //std::cout<<"setting visited to false for the link: " << link_name<<std::endl;
        robot_state_.robot_links_[link_name].setLinkDFSVisited(false);
    }
}

void RobotModel::setDisabledEnvironmentCollision(std::vector <std::pair<std::string,std::string> > disabled_collision_pair)
{
     for(int i = 0; i < disabled_collision_pair.size(); i++)
     {
        srdf::Model::DisabledCollision disabled_pair;
        disabled_pair.link1_ = disabled_collision_pair.at(i).first;
        disabled_pair.link2_ = disabled_collision_pair.at(i).second;
        // assign the disabled collision pairs to the collision library
        world_collision_detector_->AbstractCollisionDetection::disabled_collisions_.push_back(disabled_pair);
     } 
}

bool RobotModel::initializeLinksCollisions()
{     
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;

    // get the disabled collision pairs from srdf
    std::vector<srdf::Model::DisabledCollision> disabled_collision_pairs = srdf_model_->getDisabledCollisionPairs();
    // assign the disabled collision pairs to the collision library
    robot_collision_detector_->AbstractCollisionDetection::setDisabledCollisionPairs( disabled_collision_pairs);

    int total_number_of_collision_should_be = 0;    
    std::string link_name,  abs_path_to_mesh_file,  collision_object_name;    
    Eigen::Vector3d mesh_scale;
    std::vector<urdf::CollisionSharedPtr > link_collisions;

    for(std::map<std::string, RobotLink>::iterator it = robot_state_.robot_links_.begin(); it != robot_state_.robot_links_.end(); it++)
    {
        link_name 	= it->first;
        link_collisions = it->second.getLinkCollisions();

        LOG_DEBUG("For the link:%s there are %i collision objects in this link", link_name.c_str(), link_collisions.size());
        total_number_of_collision_should_be = total_number_of_collision_should_be + link_collisions.size();      

        for(std::size_t i = 0; i < link_collisions.size(); i++ )        
        {
            collision_object_name = link_name+"_" +lexical_cast<std::string>(i);

            base::Pose collision_object_pose;
            collision_object_pose.position.x() = link_collisions.at(i)->origin.position.x;
            collision_object_pose.position.y() = link_collisions.at(i)->origin.position.y;
            collision_object_pose.position.z() = link_collisions.at(i)->origin.position.z;

            collision_object_pose.orientation.w() = link_collisions.at(i)->origin.rotation.w;
            collision_object_pose.orientation.x() = link_collisions.at(i)->origin.rotation.x;
            collision_object_pose.orientation.y() = link_collisions.at(i)->origin.rotation.y;
            collision_object_pose.orientation.z() = link_collisions.at(i)->origin.rotation.z;

            LOG_DEBUG(" The collision object name is %s with origin X=%f; Y=%f, Z=%f", collision_object_name.c_str(), 
                        link_collisions.at(i)->origin.position.x, link_collisions.at(i)->origin.position.y, link_collisions.at(i)->origin.position.z);


            if(link_collisions.at(i)->geometry->type == urdf::Geometry::MESH)
            {
                LOG_DEBUG_S<<"[initializeLinksCollisions]: Registering mesh file ";

                urdf::MeshSharedPtr urdf_mesh_ptr = urdf::static_pointer_cast <urdf::Mesh> (link_collisions.at(i)->geometry);

                mesh_scale(0) = urdf_mesh_ptr->scale.x;
                mesh_scale(1) = urdf_mesh_ptr->scale.y;
                mesh_scale(2) = urdf_mesh_ptr->scale.z;

                urdf_directory_path = urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

                abs_path_of_mesh_file = resolve_path( urdf_mesh_ptr->filename, urdf_directory_path );

                abs_path_to_mesh_file=abs_path_of_mesh_file.string();

                if(!robot_collision_detector_->registerMeshToCollisionManager(abs_path_to_mesh_file, mesh_scale, 
                                                                              collision_object_name, collision_object_pose, link_padding_))
                {
                    return false;                    
                }
            }
            else if(link_collisions.at(i)->geometry->type == urdf::Geometry::BOX)
            {
                LOG_DEBUG_S<<"[initializeLinksCollisions]: Registering box ";

                urdf::BoxSharedPtr urdf_box_ptr = urdf::static_pointer_cast <urdf::Box> (link_collisions.at(i)->geometry);
                robot_collision_detector_->registerBoxToCollisionManager(urdf_box_ptr->dim.x, urdf_box_ptr->dim.y, urdf_box_ptr->dim.z,
                                                                         collision_object_name, collision_object_pose, link_padding_);
            }
            else if(link_collisions.at(i)->geometry->type == urdf::Geometry::CYLINDER)
            {
                LOG_DEBUG_S<<"[initializeLinksCollisions]: Registering cylinder ";

                urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (link_collisions.at(i)->geometry);                
                robot_collision_detector_->registerCylinderToCollisionManager(urdf_cylinder_ptr->radius, urdf_cylinder_ptr->length, 
                                                                              collision_object_name, collision_object_pose, link_padding_);
            }
            else if(link_collisions.at(i)->geometry->type == urdf::Geometry::SPHERE)
            {
                LOG_DEBUG_S<<"[initializeLinksCollisions]: Registering sphere ";

                urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (link_collisions.at(i)->geometry);                
                robot_collision_detector_->registerSphereToCollisionManager(urdf_sphere_ptr->radius,collision_object_name, collision_object_pose, link_padding_);
            }
        }
    }
    LOG_DEBUG_S<<"[initializeLinksCollisions]: Initialisation completed with "<<robot_collision_detector_->numberOfObjectsInCollisionManger()<< 
                 " collision objects and "<<robot_collision_detector_->disabled_collisions_.size()<< " disabled collision pair";

    return true;
}


boost::filesystem::path RobotModel::resolve_path( const boost::filesystem::path& p, const boost::filesystem::path& base )
{
    boost::filesystem::path abs_p = boost::filesystem::absolute(p,base);
    boost::filesystem::path result;
    for(boost::filesystem::path::iterator it=abs_p.begin(); it!=abs_p.end(); ++it)
    {
        if(*it == "..")
        {
            // /a/b/.. is not necessarily /a if b is a symbolic link
            if(boost::filesystem::is_symlink(result) )
                result /= *it;
            // /a/b/../.. is not /a/b/.. under most circumstances
            // We can end up with ..s in our result because of symbolic links
            else if(result.filename() == "..")
                result /= *it;
            // Otherwise it should be safe to resolve the parent
            else
                result = result.parent_path();
        }
        else if(*it == ".")
        {
            // Ignore
        }
        else
        {
            // Just cat other path entries
            result /= *it;
        }
    }
    return result;
}

bool RobotModel::getJointLimits(std::vector< double > &lower_limits, std::vector< double > &upper_limits)
{
    std::vector< std::pair<std::string, urdf::Joint> > planning_groups_joints_names;
    std::string base_link, tip_link;
    if(!getPlanningGroupJointinformation(planning_group_name_, planning_groups_joints_names, base_link, tip_link))
        return false;
    //lower_limits.resize(planning_groups_joints_names.size());
    //upper_limits.resize(planning_groups_joints_names.size());
    lower_limits.clear();
    upper_limits.clear();

    for(auto it = planning_groups_joints_names.begin(); it != planning_groups_joints_names.end(); it++)
    {
        lower_limits.push_back(it->second.limits->lower)  ;
        upper_limits.push_back(it->second.limits->upper)  ;	
    }

    assert(lower_limits.size() == planning_groups_joints_names.size());

    return true;
}


bool RobotModel::getJointLimits(base::JointLimits &limits)
{
    std::vector< std::pair<std::string, urdf::Joint> > planning_groups_joints_names;
    std::string base_link, tip_link;
    if(!getPlanningGroupJointinformation(planning_group_name_, planning_groups_joints_names, base_link, tip_link))
        return false;

    limits.clear();

    for(auto it = planning_groups_joints_names.begin(); it != planning_groups_joints_names.end(); it++)
    {
        limits.names.push_back(it->first);

        base::JointLimitRange range;
        range.min.position      =  it->second.limits->lower;
        range.max.position      =  it->second.limits->upper;
        range.min.speed         = -it->second.limits->velocity;
        range.max.speed         =  it->second.limits->velocity;
        range.min.effort        = -it->second.limits->effort;
        range.max.effort        =  it->second.limits->effort;

        limits.elements.push_back(range);
    }
    assert(limits.size() == planning_groups_joints_names.size());
    return true;
}



/*
Manipulability Analysis

http://h2t.anthropomatik.kit.edu/pdf/Vahrenkamp2012c.pdf

With Yoshikawa’s manipulability index [3] a quality measure for redundant manipulators was introduced, which de-
scribes the distance to singular configurations.
*/

void RobotModel::manipulabilityIndex(KDL::Jacobian  &jacobian, double &manipulability_index)
{
    Eigen::Matrix<double,6,6> jacobian_multiple_by_jacobian_inverse=jacobian.data*jacobian.data.transpose();
    manipulability_index=jacobian_multiple_by_jacobian_inverse.determinant();
}


void RobotModel::computeJacobain(const std::string &chain_root_link,const  std::string& tip_link, std::map<std::string, double> joints_name_values, KDL::Jacobian  &jacobian)
{
//#define COMPUTEJACOBAIN_LOG
    KDL::Chain kdl_chain;
    kdl_tree_.getChain(chain_root_link , tip_link , kdl_chain);
    std::string joint_name;
    double joint_value;
    KDL::JntArray kdl_chain_joint_array;
    KDL::Joint::JointType joint_type;
    int j=0;
    kdl_chain_joint_array.resize(joints_name_values.size() );
    for(std::size_t i=0;i<kdl_chain.getNrOfSegments();i++ )
    {
        joint_name=kdl_chain.getSegment(i).getJoint().getName();
        joint_type= kdl_chain.getSegment(i).getJoint().getType();
        joint_value=joints_name_values[joint_name];
        if(joint_type !=KDL::Joint::None )
        {
            kdl_chain_joint_array.data[j]=joint_value;
            j++;
        }

    }
//    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_(new KDL::ChainJntToJacSolver(kdl_chain_));

    jacobian.resize(j);

    KDL::ChainJntToJacSolver jnt_to_jac_solver_(kdl_chain);
    jnt_to_jac_solver_.JntToJac( kdl_chain_joint_array, jacobian);

    #ifdef COMPUTEJACOBAIN_LOG
    for(std::size_t i=0;i<jacobian.rows();i++ )
    {
        for(std::size_t j=0;j<jacobian.columns();j++)
        {
            std::cout<<"jacobian(i,j): " << jacobian(i,j)<<std::endl;
        }
    }
    #endif
}

bool RobotModel::getPlanningGroupBaseTipName(const std::string &planningGroupName,  std::string &base_link, std::string &tip_link)
{
    //    planning_groups_joints are in  order from base to tip! very important

    std::vector<srdf::Model::Group> srdf_groups= this->srdf_model_->getGroups();
    srdf::Model::Group planning_group;

    for(std::size_t i=0;i<srdf_groups.size();i++)
    {
        if( planningGroupName.compare(srdf_groups.at(i).name_) ==0 )
        {
            planning_group=srdf_groups.at(i);
            break;
        }
    }

    for(std::size_t i=0;i<planning_group.chains_.size();i++)
    {
        base_link = planning_group.chains_.at(i).first;
        tip_link = planning_group.chains_.at(i).second;
    }

    if(!kdl_tree_.getChain(base_link, tip_link , kdl_chain_))
        return false;
    return true;
}

bool RobotModel::getPlanningGroupJointinformation(const std::string planningGroupName,
                                                  std::vector< std::pair<std::string,urdf::Joint> > &planning_groups_joints,
                                                  std::string &base_link, std::string &tip_link)
{
    
    if(!getPlanningGroupBaseTipName(planningGroupName,  base_link, tip_link))
        return false;

    std::string joint_name;
    
    urdf::Joint planning_group_joint;

    for(std::size_t i=0;i<kdl_chain_.segments.size();i++ )
    {
        //KDL JointType: RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None;
        if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )
        {
            joint_name=kdl_chain_.getSegment(i).getJoint().getName();
            planning_group_joint=*(urdf_model_->getJoint(joint_name).get());

            planning_groups_joints.push_back(std::make_pair(joint_name ,planning_group_joint )   );
        }
    }
    
    return true;
}

bool RobotModel::getPlanningGroupJointinformation(const std::string planningGroupName,  base::samples::Joints &planning_groups_joints)
{
    std::string joint_name;
    std::string base_link, tip_link;
   
    if(!getPlanningGroupBaseTipName(planningGroupName,  base_link, tip_link))
        return false;

    urdf::Joint planning_group_joint;    
       

    for(std::size_t i=0;i<kdl_chain_.segments.size();i++ )
    {
        //KDL JointType: RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None;
        if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )
        {
            joint_name=kdl_chain_.getSegment(i).getJoint().getName();
            planning_group_joint=*(urdf_model_->getJoint(joint_name).get());

            planning_groups_joints.names.push_back(joint_name);
            base::JointState joint_state;
            joint_state.position = getRobotState().robot_joints_[joint_name].getJointValue();
            
            planning_groups_joints.elements.push_back(joint_state);            
        }
    }
    
    return true;
}

void RobotModel::getPlanningGroupJointsName(const std::string planningGroupName,
                                            std::vector< std::string> &planning_group_joints_name)
{
    //    planning_groups_joints are in  order from base to tip! very important
    std::string base_link, tip_link;
    std::vector< std::pair<std::string,urdf::Joint> > planning_group_joints;

    getPlanningGroupJointinformation(planningGroupName, planning_group_joints, base_link, tip_link);

    planning_group_joints_name.clear();

    for(std::vector< std::pair<std::string, urdf::Joint> >::iterator it= planning_group_joints.begin(); it!=planning_group_joints.end();it++)    
        planning_group_joints_name.push_back(it->first);
}

void RobotModel::setSRDF(boost::shared_ptr<srdf::Model> &srdf_model_)
{
    this->srdf_model_=srdf_model_;
}

void RobotModel::setURDF(urdf::ModelInterfaceSharedPtr &urdf_model_)
{
    this->urdf_model_=urdf_model_;
}

boost::shared_ptr<srdf::Model>  const & RobotModel::getSRDF()
{
    return this->srdf_model_;
}

urdf::ModelInterfaceSharedPtr const &  RobotModel::getURDF()
{
    return this->urdf_model_;
}



void RobotModel::addGraspObject(urdf::CollisionSharedPtr grasp_object, std::string parent_link_name)
{
    //#define ADDGRASPOBJECT_LOG

    #ifdef ADDGRASPOBJECT_LOG
        std::cout<<" ==Start ADD grasp object =============================="<<std::endl;
    #endif

    // grasp object should have unique name.
    if (urdf_model_->links_.find(grasp_object->name) != urdf_model_->links_.end())
    {
        std::cout<<"Please give the grasp object a unique name. Grasp object with name "<<grasp_object->name<<" is already available"<<std::endl;
        return;
    }

    // creating a urdf link for the grasp object
    urdf::LinkSharedPtr grasp_link;
    grasp_link.reset(new urdf::Link);
    grasp_link->name = grasp_object->name;
    grasp_link->collision = grasp_object;

    // add the grasp object to urdf model
    urdf_model_->links_.insert(make_pair(grasp_link->name, grasp_link));

    // set the parent and child for the grasp link
    urdf::LinkSharedPtr parent_link;
    urdf_model_->getLink(parent_link_name, parent_link);

    //set parent link for grasp link
    grasp_link->setParent(parent_link);

    //set parent joint for grasp link
    urdf::JointSharedPtr grasp_object_joint;
    grasp_object_joint.reset(new urdf::Joint);

    grasp_object_joint->name = "grasp_object_joint" ;
    grasp_object_joint->child_link_name = grasp_object->name ;
    grasp_object_joint->parent_link_name = parent_link_name;
    grasp_object_joint->type = urdf::Joint::FIXED;
    /*grasp_object_joint->axis.x=0;grasp_object_joint->axis.y=0;grasp_object_joint->axis.z=1;
    grasp_object_joint->parent_to_joint_origin_transform.position.x=0;
    grasp_object_joint->parent_to_joint_origin_transform.position.y=0;
    grasp_object_joint->parent_to_joint_origin_transform.position.z=0;
    grasp_object_joint->parent_to_joint_origin_transform.rotation=urdf::Rotation(0,0,0,1);*/
    parent_link->child_links.push_back(grasp_link);
    grasp_link->parent_joint =grasp_object_joint;

    #ifdef ADDGRASPOBJECT_LOG
    std::cout<<"The following links are now available in the urdf_model"<<std::endl;
    for(std::map<std::string, urdf::LinkSharedPtr >::iterator it=urdf_model_->links_.begin(); it!=urdf_model_->links_.end();it++)
        std::cout<<it->second->name<<std::endl;
    #endif

    // creating a robot link for the given grasp object
    RobotLink robot_link;
    robot_link.setLinkName(grasp_object->name);
    robot_state_.robot_links_[grasp_object->name] = robot_link;

    robot_state_.robot_links_[grasp_object->name].setLinkCollision(grasp_object );
    robot_state_.robot_links_[grasp_object->name].setLinkDFSVisited(false);

    // setting frame for the grasp object
    KDL::Frame parent_link_in_base_link = robot_state_.robot_links_[parent_link_name].getLinkFrame();
    KDL::Frame grasp_object_frame_in_parent_frame;
    KDL::Frame grasp_object_frame_in_base_link;

    grasp_object_frame_in_parent_frame.p.data[0] = grasp_object->origin.position.x;
    grasp_object_frame_in_parent_frame.p.data[1] = grasp_object->origin.position.y;
    grasp_object_frame_in_parent_frame.p.data[2] = grasp_object->origin.position.z;

    grasp_object_frame_in_parent_frame.M = KDL::Rotation::Quaternion(grasp_object->origin.rotation.x, grasp_object->origin.rotation.y,
                                            grasp_object->origin.rotation.z, grasp_object->origin.rotation.w);

    grasp_object_frame_in_base_link = parent_link_in_base_link * grasp_object_frame_in_parent_frame;


    robot_state_.robot_links_[grasp_object->name].setLinkFrame(grasp_object_frame_in_base_link);

    
    //robot_state_.robot_links_[grasp_object->name].calculateLinkVisualsPoseInGlobalPose();
    robot_state_.robot_links_[grasp_object->name].calculateLinkCollisionPoseinGlobalPose();

    // disable the collision between the grasp object and its parent link
    srdf::Model::DisabledCollision disable_collision_grasp_object_w_parent_link;
    disable_collision_grasp_object_w_parent_link.link1_ = grasp_object->name;
    disable_collision_grasp_object_w_parent_link.link2_ = parent_link_name;
    disable_collision_grasp_object_w_parent_link.reason_ = "Adjacent";

    robot_collision_detector_->AbstractCollisionDetection::addDisabledCollisionPairs(disable_collision_grasp_object_w_parent_link);

    // register the grasp object to the collision manager.
    double grasp_rot_x = 0.0, grasp_rot_y = 0.0, grasp_rot_z = 0.0, grasp_rot_w = 0.0;    
    grasp_object_frame_in_base_link.M.GetQuaternion(grasp_rot_x, grasp_rot_y, grasp_rot_z, grasp_rot_w);

    /*fcl::Quaternion3f collision_quaternion_orientation (grasp_rot_x, grasp_rot_y, grasp_rot_z, grasp_rot_w);
    fcl::Vec3f collision_object_translation (grasp_object_frame_in_base_link.p.data[0],
                                             grasp_object_frame_in_base_link.p.data[1],
                                             grasp_object_frame_in_base_link.p.data[2]);
    */
    
    base::Pose collision_pose;
    collision_pose.position.x() = grasp_object_frame_in_base_link.p.data[0];
    collision_pose.position.y() = grasp_object_frame_in_base_link.p.data[1];
    collision_pose.position.z() = grasp_object_frame_in_base_link.p.data[2];
    
    collision_pose.orientation.x() = grasp_rot_x;
    collision_pose.orientation.y() = grasp_rot_y;
    collision_pose.orientation.z() = grasp_rot_z;
    collision_pose.orientation.w() = grasp_rot_w;


    // All collision object will have a suffix depending on the link collision.
    // i.e.,:   Consider a case in which a robot link_1's collision is decribed by 3 pritimive objects.
    //          The robot model stores the link collision as link_1_1, link_1_2, link_1_3. Thats how robot model
    //          is updated. Because of this issue, we need to add a string "_0" to the grasp object
    std::string grasp_object_name;
    grasp_object_name = grasp_object->name+"_0";


    if(grasp_object->geometry->type == urdf::Geometry::MESH)
    {
        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering mesh file------------------------------ " <<std::endl;
        #endif
        urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (grasp_object->geometry);

        //double scale_for_mesha_files_x=urdf_mesh_ptr->scale.x;
        //double scale_for_mesha_files_y=urdf_mesh_ptr->scale.y;
        //double scale_for_mesha_files_z=urdf_mesh_ptr->scale.z;

        Eigen::Vector3d mesh_scale(urdf_mesh_ptr->scale.x, urdf_mesh_ptr->scale.y, urdf_mesh_ptr->scale.z);

        boost::filesystem::path relative_path_of_mesh_file_in_urdf_file=urdf_mesh_ptr->filename;

        std::string urdf_directory_path=urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

        boost::filesystem::path abs_path_of_mesh_file =resolve_path( relative_path_of_mesh_file_in_urdf_file, urdf_directory_path );

        std::string abs_path_to_mesh_file = abs_path_of_mesh_file.string();
        robot_collision_detector_->registerMeshToCollisionManager(abs_path_to_mesh_file,mesh_scale, grasp_object_name, collision_pose, link_padding_);
    }
    else if(grasp_object->geometry->type == urdf::Geometry::BOX)
    {

        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering box------------------------------ " <<std::endl;
        #endif

        urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (grasp_object->geometry);

        double x = urdf_box_ptr->dim.x;
        double y = urdf_box_ptr->dim.y;
        double z = urdf_box_ptr->dim.z;

        robot_collision_detector_->registerBoxToCollisionManager( x,y,z, grasp_object_name, collision_pose, link_padding_);
    }
    else if(grasp_object->geometry->type == urdf::Geometry::CYLINDER)
    {

        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering cylinder------------------------------" <<std::endl;
        #endif

        urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (grasp_object->geometry);
        double radius = urdf_cylinder_ptr->radius;
        double length = urdf_cylinder_ptr->length;
        robot_collision_detector_->registerCylinderToCollisionManager(radius, length, grasp_object_name,
                                                                                collision_pose,link_padding_);
    }
    else if(grasp_object->geometry->type == urdf::Geometry::SPHERE)
    {
        #ifdef ADDGRASPOBJECT_LOG
            std::cout<<"------------------------------registering sphere------------------------------ " <<std::endl;
        #endif

        urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (grasp_object->geometry);
        double radius = urdf_sphere_ptr->radius;
        robot_collision_detector_->registerSphereToCollisionManager(radius,grasp_object_name, collision_pose,link_padding_  );
    }

    #ifdef ADDGRASPOBJECT_LOG
    std::cout<<""<<std::endl;
    std::cout<<"The following links are now available in the robot_state"<<std::endl;
    for(std::map<std::string, RobotLink>::iterator it=robot_state_.robot_links.begin();it!=robot_state_.robot_links.end();it++  )
        std::cout<<it->first<<std::endl;
    std::cout<<" ==============================End ADD grasp object=="<<std::endl;
    #endif

}

bool RobotModel::removeGraspObject(const std::string grasp_object_name)
{

//     LOG_DEBUG_S<<"[RobotModel]: The following links are available in the urdf_model before removing the link %s, grasp_object_name.c_str()";
//     for(std::map<std::string, urdf::LinkSharedPtr >::iterator it=urdf_model_->links_.begin(); it!=urdf_model_->links_.end();it++)
//     {
//         std::cout<<it->second->name<<std::endl;
//     }
//     std::cout<<""<<std::endl;
    
    // remove grasp link from urdf model
    urdf::LinkConstSharedPtr link_ptr;
    if (urdf_model_->links_.find(grasp_object_name) == urdf_model_->links_.end())
    {
        std::cout<<" No grasp object is available with name "<< grasp_object_name<<std::endl;
        return false;
    }
    else
    {
        // get the pointer of the grasp link in the urdf model
        link_ptr = urdf_model_->links_.find(grasp_object_name)->second;

        //detach or remove from its parent link
        urdf::LinkSharedPtr parent_link;
        parent_link = link_ptr->getParent();
        _removeLinkName = grasp_object_name;
        parent_link->child_links.erase(std::remove_if(parent_link->child_links.begin(), parent_link->child_links.end(), isLinkListed),parent_link->child_links.end() );

        // now remove the grasp link from the urdf model
        urdf_model_->links_.erase(grasp_object_name);
    }

//     std::cout<<"The following links are available in the urdf_model after removing the link "<<grasp_object_name<<std::endl;
//     for(std::map<std::string, urdf::LinkSharedPtr >::iterator it=urdf_model_->links_.begin(); it!=urdf_model_->links_.end();it++)
//     {
//         std::cout<<it->second->name<<std::endl;
//     }
    
    // remove grasp link from robot model
    robot_state_.robot_links_.erase(grasp_object_name);

    // remove the grasp link form collision pair    
    robot_collision_detector_->removeDisabledCollisionLink(grasp_object_name);

     // remove the grasp link from collision data base
    return robot_collision_detector_->removeSelfCollisionObject(grasp_object_name);

}

bool RobotModel::removeWorldObject(const std::string world_object_name)
{
    return world_collision_detector_->removeWorldCollisionObject(world_object_name);
}

void RobotModel::updateJoint(std::string joint_name, double joint_value)
{ 
    if(this->urdf_model_->getJoint(joint_name) )
    {
        if(! robot_state_.robot_joints_[joint_name].isMimicJoint() )
        {
     
            std::string start_link_name= this->urdf_model_->getJoint(joint_name)->parent_link_name;

            if(robot_state_.robot_joints_[joint_name].mimic_joints_map_.size() > 0)
            {
                for (std::map< std::string, MimicJoint >::iterator it = robot_state_.robot_joints_[joint_name].mimic_joints_map_.begin();
                     it!= robot_state_.robot_joints_[joint_name].mimic_joints_map_.end(); ++it)
                {
                    robot_state_.robot_joints_[it->first].setJointValue ( (joint_value * it->second.multiplier) + it->second.offset);
                }
            }
            else
                robot_state_.robot_joints_[joint_name].setJointValue (joint_value);

            std::vector<std:: string> visited_links;

            dfsTraversing(start_link_name, visited_links);

            settingVisitedFlagLinkToFalse(visited_links);
            //std::cout<<"list of visited link for joint "<<joint_name.c_str()<<"  "<<visited_links.size()<<std::endl;
            LOG_DEBUG_S<<"========================== list of visited link for joint "<<joint_name.c_str();

            /*for(std::size_t i=0;i<visited_links.size();i++)
            {
                LOG_DEBUG_S<<visited_links.at(i).c_str();
            }*/

            std::string name_of_visited_link, name_of_collision_object;
            //double quaternion_w,quaternion_x,quaternion_y,quaternion_z, translation_x,translation_y,translation_z;
            base::Pose collision_object_pose;


            ////////////////////link visual and collision pose has been updated in dfstraverse, but we have to update teh collision manger //////////////////////////////////
            //for(std::size_t i=0;i<visited_links.size();i++)
            for(auto &vl: visited_links)
            {
                //name_of_visited_link=visited_links.at(i);
                name_of_visited_link = vl;

                //std::vector<urdf::CollisionSharedPtr >link_collisions = robot_state_.robot_links_[name_of_visited_link].getLinkCollisions() ;
                std::vector<urdf::CollisionSharedPtr >link_collisions(0);
                robot_state_.robot_links_[name_of_visited_link].getLinkCollisions(link_collisions) ;

                //for(std::size_t j=0;j<link_collisions.size();j++ )
                for(auto &lc: link_collisions)
                {
                    //urdf::CollisionSharedPtr link_collision_global_pose =link_collisions.at(j);
                    urdf::CollisionSharedPtr link_collision_global_pose = lc;

                    collision_object_pose.position.x() = link_collision_global_pose->origin.position.x;
                    collision_object_pose.position.y() = link_collision_global_pose->origin.position.y;
                    collision_object_pose.position.z() = link_collision_global_pose->origin.position.z;

                    collision_object_pose.orientation.w() = link_collision_global_pose->origin.rotation.w;
                    collision_object_pose.orientation.x() = link_collision_global_pose->origin.rotation.x;
                    collision_object_pose.orientation.y() = link_collision_global_pose->origin.rotation.y;
                    collision_object_pose.orientation.z() = link_collision_global_pose->origin.rotation.z;


                    //name_of_collision_object=name_of_visited_link+"_"+lexical_cast<std::string>(j);
                    name_of_collision_object=name_of_visited_link+"_"+lexical_cast<std::string>(&lc - &link_collisions[0]);
                    robot_collision_detector_->updateCollisionObjectTransform(name_of_collision_object, collision_object_pose);
                }

            }

        }
    }
}

void RobotModel::updateJointGroup(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values)
{
    assert(joint_names.size() == joint_values.size());	    
    //auto start_time = std::chrono::high_resolution_clock::now(); 

    //for(std::size_t i=0;i<joint_names.size();i++)
    //    this->updateJoint(joint_names.at(i) ,joint_values(i) ) ;
    
    for(auto &jn: joint_names)
    {
        auto i = &jn - &joint_names[0];
        this->updateJoint(joint_names.at(i) ,joint_values(i) ) ;
    }
    
    //auto finish_time = std::chrono::high_resolution_clock::now();
   //std::chrono::duration<double> elapsed = finish_time - start_time;
   //std::cout << "Elapsed time colli: " << elapsed.count() << " s\n";
}

void RobotModel::updateJointGroup(const base::samples::Joints &joint_values)
{
    for(std::size_t i = 0; i < joint_values.size(); i++) 
        updateJoint(joint_values.names[i], joint_values.elements[i].position) ;

}

void RobotModel::updateJointGroup(const std::map< std::string ,double > &joint_values)
{
    for(std::map<std::string,double>::const_iterator it=joint_values.begin();it!=joint_values.end();++it) 
        this->updateJoint(it->first, it->second) ;

}

void RobotModel::updateJointGroup( const std::vector<std::string> &joint_names, const std::vector<double> &joint_values)
{
    std::string joint_name;
    for(std::size_t i=0;i<joint_names.size();i++)
    {
        joint_name=joint_names.at(i);
        this->updateJoint(joint_name ,joint_values.at(i) ) ;
    }
}

void RobotModel::updateOctomap(const std::shared_ptr<octomap::OcTree> &octomap, std::string collision_object_name)
{
    world_collision_detector_->updateEnvironment(octomap, collision_object_name);
}

void RobotModel::assignPlanningScene(   const std::shared_ptr<octomap::OcTree> &octomap, const std::string &link_name, std::string collision_object_name)
{

    if(collision_object_name.empty())
        collision_object_name = link_name+"_" +lexical_cast<std::string>(world_collision_detector_->numberOfObjectsInCollisionManger());

    base::Pose collision_object_pose;
    collision_object_pose.position.setZero();
    collision_object_pose.orientation.setIdentity();

    world_collision_detector_->registerOctreeToCollisionManager(octomap, collision_object_pose, collision_object_name );    
}

bool RobotModel::isStateValid(int self_collision_num_max_contacts, int external_collision_manager_num_max_contacts)
{
//     auto start_time = std::chrono::high_resolution_clock::now();

    if (robot_collision_detector_->checkSelfCollision(self_collision_num_max_contacts))
    {

        LOG_DEBUG("[RobotModel]: There is no self collision, now checking for collision against environment");        

        if(robot_collision_detector_->checkWorldCollision(external_collision_manager_num_max_contacts))
        {
            LOG_DEBUG("[RobotModel]: There is no collision against environment" );  
//             auto finish_time = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double> elapsed = finish_time - start_time;
//             std::cout<<"Col Time = "<<elapsed.count()<<std::endl;
            return true;
        }
        else
        {
            LOG_DEBUG("[RobotModel]: There is collision against environment" );
//             auto finish_time = std::chrono::high_resolution_clock::now();
//             std::chrono::duration<double> elapsed = finish_time - start_time;
//             std::cout<<"Col Time = "<<elapsed.count()<<std::endl;
            return false;
        }
    }
//     auto finish_time = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double> elapsed = finish_time - start_time;
//     std::cout<<"Col Time = "<<elapsed.count()<<std::endl;
    return false;
}


/*void RobotModel::ConvertPoseBetweenFrames( const std::string B_Frame_Name, const base::samples::RigidBodyState &F_B_C , const std::string &A_Frame_Name ,
					   base::samples::RigidBodyState &F_A_C )
{
    KDL::Frame kdl_frame_f_b_c;
    kdl_frame_f_b_c.p.data[0] = F_B_C.position(0);
    kdl_frame_f_b_c.p.data[1] = F_B_C.position(1);
    kdl_frame_f_b_c.p.data[2] = F_B_C.position(2);

    kdl_frame_f_b_c.M = KDL::Rotation::Quaternion(F_B_C.orientation.x(), F_B_C.orientation.y(),
    F_B_C.orientation.z(), F_B_C.orientation.w() );
    
    KDL::Frame kdl_frame_f_a_c; 
    ConvertPoseBetweenFrames(B_Frame_Name, kdl_frame_f_b_c , A_Frame_Name, kdl_frame_f_a_c );
    
    F_A_C.position(0) = kdl_frame_f_a_c.p.data[0];
    F_A_C.position(1) = kdl_frame_f_a_c.p.data[1];
    F_A_C.position(2) = kdl_frame_f_a_c.p.data[2];
    
    kdl_frame_f_a_c.M.GetQuaternion(F_A_C.orientation.x(),F_A_C.orientation.y(),F_A_C.orientation.z(),F_A_C.orientation.w());
}*/

void RobotModel::ConvertPoseBetweenFrames( const std::string B_Frame_Name, const KDL::Frame &F_B_C , const std::string &A_Frame_Name ,KDL::Frame &F_A_C )
{

/*
    F_A_C = F_A_B * F_B_C

    F_A_C ==> will be calculated by this function
    F_A_B ==> calculated by kdl FK, A is chain_root, B is chain_tip
    F_B_C ==> is given

    A ==> is chain_root
    B ==> is chain_tip
    C ==> given pose is in this frame

*/
//        You can use the operator * to compose frames. If you have a Frame F_A_B that expresses the pose of frame B wrt frame A,
//        and a Frame F_B_C that expresses the pose of frame C wrt to frame B, the calculation of Frame F_A_C that
//        expresses the pose of frame C wrt to frame A is as follows:
//        Frame F_A_C = F_A_B * F_B_C;

    KDL::Frame F_A_B;
    std::string chain_root_link, chain_tip_link;
    chain_root_link= A_Frame_Name;
    chain_tip_link=B_Frame_Name;
    KDL::JntArray kdl_chain_joint_array;

    kdl_tree_.getChain(chain_root_link , chain_tip_link , kdl_chain_);
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
    kdl_chain_joint_array.resize(kdl_chain_.getNrOfJoints());
    std::string link_name;
    std::string joint_name;
    int joint_type;
    int j=0;
    double joint_value;
    for(std::size_t i=0;i<kdl_chain_.segments.size();i++)
    {
        link_name=kdl_chain_.getSegment(i).getName();
        joint_name=kdl_chain_.getSegment(i).getJoint().getName();
        joint_type=this->urdf_model_->getJoint(joint_name)->type;
        
        //std::string joint_array_names[]={"UNKNOWN", "REVOLUTE", "CONTINUOUS", "PRISMATIC", "FLOATING", "PLANAR", "FIXED"};
        //std::cout<<"the link name is " <<link_name<<std::endl;
        //std::cout<<"the joint is:" << joint_name <<std::endl;
        //std::cout<<"joint type is "<<joint_array_names[joint_type] <<std::endl;
        
        if(joint_type!=urdf::Joint::FIXED)
        {
            joint_value=robot_state_.robot_joints_[joint_name].getJointValue();
            kdl_chain_joint_array.data[j]=joint_value;
            j++;            
        }
    }
    fk_solver.JntToCart(kdl_chain_joint_array, F_A_B);
    F_A_C = F_A_B * F_B_C;
}

void RobotModel::getRobotCollisions(std::vector<urdf::CollisionSharedPtr > &  robotCollisions)
{
    // this will return the Collisons (defined in the urdf) of robot in global pose
    std::vector<urdf::CollisionSharedPtr > link_collisions;
    for(std::map<std::string, RobotLink> ::iterator it=robot_state_.robot_links_.begin();it!=robot_state_.robot_links_.end();it++)
    {
        link_collisions=it->second.getLinkCollisions();
        for(std::size_t i=0;i<link_collisions.size();i++)
        {
           robotCollisions.push_back( link_collisions.at(i) );
        }
    }
}

void RobotModel::getRobotVisuals(std::vector<urdf::VisualSharedPtr > &  robotVisuals)
{
    // this will return the Collisons (defined in the urdf) of robot in global pose
    std::vector<urdf::VisualSharedPtr > link_Visuals;
    for(std::map<std::string, RobotLink> ::iterator it=robot_state_.robot_links_.begin();it!=robot_state_.robot_links_.end();it++)
    {
        link_Visuals=it->second.getLinkVisuals();
        for(std::size_t i=0;i<link_Visuals.size();i++)
        {
           robotVisuals.push_back( link_Visuals.at(i) );
        }
    }
}

void RobotModel::addCollisionsToWorld(urdf::CollisionSharedPtr &robotCollision, std::string link_name)
{    
    
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;

    if(robotCollision->name.empty())
    {
        robotCollision->name=link_name+"_" +lexical_cast<std::string>(world_collision_detector_->numberOfObjectsInCollisionManger());
    }
    
    LOG_DEBUG_S<<"[addCollisionsToWorld]: Collision object name is "<<robotCollision->name;
        
    base::Pose collision_object_pose;
    collision_object_pose.position.x() = robotCollision->origin.position.x;
    collision_object_pose.position.y() = robotCollision->origin.position.y;
    collision_object_pose.position.z() = robotCollision->origin.position.z;
    collision_object_pose.orientation.x() = robotCollision->origin.rotation.x;
    collision_object_pose.orientation.y() = robotCollision->origin.rotation.y;
    collision_object_pose.orientation.z() = robotCollision->origin.rotation.z;
    collision_object_pose.orientation.w() = robotCollision->origin.rotation.w;

    if(robotCollision->geometry->type == urdf::Geometry::MESH)
    {
        LOG_DEBUG_S<<"[addCollisionsToWorld]: registering mesh file";

        urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (robotCollision->geometry);

        Eigen::Vector3d mesh_scale(urdf_mesh_ptr->scale.x, urdf_mesh_ptr->scale.y, urdf_mesh_ptr->scale.z);            

        urdf_directory_path = urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );	

        abs_path_of_mesh_file =resolve_path( urdf_mesh_ptr->filename, urdf_directory_path );

        world_collision_detector_->registerMeshToCollisionManager(abs_path_of_mesh_file.string(), mesh_scale, robotCollision->name, collision_object_pose, link_padding_);
    }
    else if(robotCollision->geometry->type == urdf::Geometry::BOX)
    {
        LOG_DEBUG_S<<"[addCollisionsToWorld]: registering box";

        urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (robotCollision->geometry);

        world_collision_detector_->registerBoxToCollisionManager( urdf_box_ptr->dim.x, urdf_box_ptr->dim.y, urdf_box_ptr->dim.z,robotCollision->name, collision_object_pose, link_padding_);
    }
    else if(robotCollision->geometry->type == urdf::Geometry::CYLINDER)
    {
	LOG_DEBUG_S<<"[addCollisionsToWorld]: registering cylinder";

        urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (robotCollision->geometry);
        
        world_collision_detector_->registerCylinderToCollisionManager(urdf_cylinder_ptr->radius, urdf_cylinder_ptr->length, robotCollision->name,collision_object_pose,link_padding_);
    }
    else if(robotCollision->geometry->type == urdf::Geometry::SPHERE)
    {
        LOG_DEBUG_S<<"[addCollisionsToWorld]: registering sphere";

        urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (robotCollision->geometry);

        world_collision_detector_->registerSphereToCollisionManager(urdf_sphere_ptr->radius, robotCollision->name, collision_object_pose ,link_padding_  );
    }
}

void RobotModel::generateRandomJointValue(const std::string  &planningGroupName,std::map<std::string, double  >   &planning_groups_joints_with_random_values)
{
    std::vector< std::pair<std::string,urdf::Joint >  >   planning_groups_joints;

    std::string base_link, tip_link;
    this->getPlanningGroupJointinformation(planningGroupName , planning_groups_joints, base_link,  tip_link);
    for(std::vector< std::pair<std::string,urdf::Joint >  >::iterator it=planning_groups_joints.begin();it!=planning_groups_joints.end();it++ )
    {
        double random_joint_value= randomFloat(it->second.limits->lower,it->second.limits->upper);
        planning_groups_joints_with_random_values[it->first]=random_joint_value;
    }
    return;
}

float RobotModel::randomFloat(const float& min,const  float &max)
{
    srand(time(NULL));
    float r = (float)rand() / (float)RAND_MAX;
    return min + r * (max - min);
}



void RobotModel::printWorldCollisionObject()
{
     world_collision_detector_->printCollisionObject();
}

bool RobotModel::getChainJointState(std::string base_link, std::string tip_link,
                                                  std::map< std::string, double > &planning_groups_joints)
{
    //    planning_groups_joints are in  order from base to tip! very important
    if(!kdl_tree_.getChain(base_link, tip_link , kdl_chain_))
    return false;
    for(std::size_t i=0;i<kdl_chain_.segments.size();i++ )
    {
        //KDL JointType: RotAxis,RotX,RotY,RotZ,TransAxis,TransX,TransY,TransZ,None;
        if(! (kdl_chain_.getSegment(i).getJoint().getType()==KDL::Joint::None) )
        {
            std::string joint_name=kdl_chain_.getSegment(i).getJoint().getName();
            double joint_state = getRobotState().robot_joints_[joint_name].getJointValue();
            planning_groups_joints[joint_name] = joint_state;
        }
    }

    return true;
}

void RobotModel::getLinkTransformByName(const std::string link_name, Eigen::Vector3d &position, Eigen::Vector4d &orientation)
{

    KDL::Frame link = robot_state_.robot_links_[link_name].getLinkFrame();

    position.x() = link.p[0];
    position.y() = link.p[1];
    position.z() = link.p[2];

    link.M.GetQuaternion(orientation.x(), orientation.y(), orientation.z(), orientation.w());

}

bool RobotModel::getRobotCollisionInfo(std::vector<collision_detection::DistanceInformation> &contact_info)
{
    bool no_collision = isStateValid();
    if(!no_collision)
    {
        contact_info = robot_collision_detector_->getSelfContacts();
        contact_info.insert(contact_info.end(), robot_collision_detector_->getEnvironmentalContacts().begin(), robot_collision_detector_->getEnvironmentalContacts().end());
    }
    return no_collision;
}

void RobotModel::getRobotDistanceToCollisionInfo(std::vector<collision_detection::DistanceInformation> &distance_info)
{
    robot_collision_detector_->computeSelfDistanceInfo();
    distance_info = robot_collision_detector_->getSelfDistanceInfo();
    robot_collision_detector_->computeClosestObstacleToRobotDistanceInfo();
    distance_info.insert(distance_info.end(), robot_collision_detector_->getClosestObstacleToRobotDistanceInfo().begin(), robot_collision_detector_->getClosestObstacleToRobotDistanceInfo().end());


}

}// end namespace 



/* Subtracting the pointcloud using this method is not efficient because it creates a convex hull for the entire robot.
 * In doing so, computation time is less but it will delete all the points between the robot links.
 *
 * The commented functions are not used due to the reason mentioned above and also self-filter should be done outside the motion planner
 * Once in a while we also had issues with PCL and c++11. So due to the reasons mentioned above, functions related to plc based self filter are commented.
 */

/*
template<class urdfT>
void RobotModel::registerLinks(const urdfT &urdf_link, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud )
{   
    boost::filesystem::path abs_path_of_mesh_file;
    std::string urdf_directory_path;

    std::string abs_path_to_mesh_file;
    
    std::shared_ptr<collision_detection::MeshLoader> mesh_loader(new collision_detection::MeshLoader());    

    for(std::size_t i=0;i<urdf_link.size();i++ )
    {
        if(urdf_link.at(i)->geometry->type == urdf::Geometry::MESH)
        {    
            LOG_DEBUG_S<<"[registerLinks]: Registering mesh file ";         
           
            urdf::MeshSharedPtr urdf_mesh_ptr= urdf::static_pointer_cast <urdf::Mesh> (urdf_link.at(i)->geometry);

            urdf_directory_path=urdf_file_abs_path_.substr(0, urdf_file_abs_path_.find_last_of("/") );

            abs_path_of_mesh_file =resolve_path( urdf_mesh_ptr->filename, urdf_directory_path );

            abs_path_to_mesh_file=abs_path_of_mesh_file.string();
            pcl::PointCloud<pcl::PointXYZ> point_cloud;

            mesh_loader->createPointCloudFromMesh(abs_path_to_mesh_file, point_cloud, urdf_mesh_ptr->scale.x, urdf_mesh_ptr->scale.y, urdf_mesh_ptr->scale.z);
            link_point_cloud.push_back(point_cloud);
        }
        else if(urdf_link.at(i)->geometry->type == urdf::Geometry::BOX)
        {
            LOG_DEBUG_S<<"[registerLinks]: Registering box ";

            urdf::BoxSharedPtr urdf_box_ptr= urdf::static_pointer_cast <urdf::Box> (urdf_link.at(i)->geometry);

            pcl::PointCloud<pcl::PointXYZ> point_cloud;

            createPtCloudFromBox(point_cloud,urdf_box_ptr->dim.x, urdf_box_ptr->dim.y, urdf_box_ptr->dim.z);
            link_point_cloud.push_back(point_cloud);
        }
        else if(urdf_link.at(i)->geometry->type == urdf::Geometry::CYLINDER)
        {
            LOG_DEBUG_S<<"[registerLinks]: Registering cylinder ";

            urdf::CylinderSharedPtr urdf_cylinder_ptr= urdf::static_pointer_cast <urdf::Cylinder> (urdf_link.at(i)->geometry);

            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            createPtCloudFromCylinder(point_cloud, urdf_cylinder_ptr->radius, urdf_cylinder_ptr->length);
            link_point_cloud.push_back(point_cloud);
        }
        else if(urdf_link.at(i)->geometry->type == urdf::Geometry::SPHERE)
        {
            LOG_DEBUG_S<<"[registerLinks]: Registering sphere ";            

            urdf::SphereSharedPtr urdf_sphere_ptr= urdf::static_pointer_cast <urdf::Sphere> (urdf_link.at(i)->geometry);

            pcl::PointCloud<pcl::PointXYZ> point_cloud;
            createPtCloudFromSphere(point_cloud, urdf_sphere_ptr->radius);
            link_point_cloud.push_back(point_cloud);
        }
    }
}

void RobotModel::scalePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out, double scale_x, double scale_y, double scale_z)
{
    double cloud_in_x,cloud_in_y,cloud_in_z;

    for(std::size_t i=0; i < cloud_in->size();i++)
    {
        cloud_in_x=cloud_in->at(i).x;
        cloud_in_y=cloud_in->at(i).y;
        cloud_in_z=cloud_in->at(i).z;
        cloud_out->push_back(pcl::PointXYZ(cloud_in_x /scale_x,cloud_in_y /scale_y, cloud_in_z /scale_z) );
    }
}

void RobotModel::createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,
                                            double x, double y, double z, Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix, bool dense)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr box_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()) ;
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,-z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,z/2 ) );
    box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,-z/2 ) );

    if(dense)
    {
        double delta_x= 0.02;
        double delta_y= 0.02;
        double delta_z= 0.02;

        for(double deltaX = -x/2 + delta_x; deltaX < x/2; deltaX = deltaX+delta_x)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,-z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,-z/2 ) );

            for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,-y/2 ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,y/2 ,deltaZ ) );

            }

            for(double deltaY = -y/2 + delta_y; deltaY < y/2; deltaY = deltaY+delta_y)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,-z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,z/2 ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(deltaX ,deltaY ,-z/2 ) );}

        }

        for(double deltaY = -y/2 + delta_y; deltaY < y/2; deltaY = deltaY+delta_y)
        {
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,-z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,z/2 ) );
            box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,-z/2 ) );

            for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
            {
                box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,deltaZ ) );
                box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,deltaY ,deltaZ ) );
            }
        }
//         for(double deltaZ = -z/2 + delta_z; deltaZ < z/2; deltaZ = deltaZ+delta_z)
//         {
//             box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,y/2 ,deltaZ ) );
//             box_cloud_ptr->push_back(pcl::PointXYZ(x/2 ,-y/2 ,deltaZ ) );
//             box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,-y/2 ,deltaZ ) );
//             box_cloud_ptr->push_back(pcl::PointXYZ(-x/2 ,y/2 ,deltaZ ) );
//         }
    }

    pcl::transformPointCloud (*box_cloud_ptr, *transformed_box_cloud_ptr, link_visual_pose_in_sensor_frame_eigen_matrix);
}

void RobotModel::createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cylinder_cloud_ptr,
                                                 double radius, double height, Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix,
                                                 int number_of_step_alpha, bool dense )
{
    double alpha_angle=M_PI/number_of_step_alpha;
    double z=height/2;
    double x,y;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

    for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
    {
        x=radius*cos(alpha);
        y=radius*sin(alpha);
        cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,z));
        cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,-z));
    }

    if (dense)
    {
        double delta_z = z * 0.1;
        for(double delta = delta_z; delta < z; delta = delta+delta_z)
        {
            for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
            {
                x=radius*cos(alpha);
                y=radius*sin(alpha);
                cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,delta));
                cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,-delta));
            }
        }
        for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
        {
            x=radius*cos(alpha);
            y=radius*sin(alpha);
            cylinder_cloud_ptr->push_back(pcl::PointXYZ(x,y,0.0));
        }
    }

    pcl::transformPointCloud (*cylinder_cloud_ptr, *transformed_cylinder_cloud_ptr, link_visual_pose_in_sensor_frame_eigen_matrix);
}

void RobotModel::createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sphere_cloud_ptr, double radius, 
                                         Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha, int number_of_step_beta)
{

    // alpha cretae point on a circle given radius, beta will give you the radius,

    double alpha_angle, beta_angle;
    double x_origin, y_origin, z_origin;

    x_origin=link_visual_pose_in_sensor_frame_eigen_matrix(0,3);
    y_origin=link_visual_pose_in_sensor_frame_eigen_matrix(1,3);
    z_origin=link_visual_pose_in_sensor_frame_eigen_matrix(2,3);


    alpha_angle= M_PI/number_of_step_alpha;
    beta_angle= M_PI/number_of_step_beta;


    double x,y,z,new_radius;

    for(double beta=0;beta<M_PI/2;beta=beta+beta_angle)
    {
        new_radius=radius*cos(beta);
        z=radius*sin(beta);

        for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
        {
            x=new_radius*cos(alpha)+x_origin;
            y=new_radius*sin(alpha)+y_origin;
            transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x,y,z+z_origin));
            transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x,y,-z+z_origin));
        }
    }

    //top and bottom of the sphere
    transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x_origin,y_origin,z_origin+radius));
    transformed_sphere_cloud_ptr->push_back(pcl::PointXYZ(x_origin,y_origin,z_origin-radius));


}

void RobotModel::createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ> &box_cloud,double x, double y, double z)
{
    box_cloud.push_back(pcl::PointXYZ(x/2 ,y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(x/2 ,y/2 ,-z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(x/2 ,-y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(x/2 ,-y/2 ,-z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,y/2 ,-z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,-y/2 ,z/2 ) );
    box_cloud.push_back(pcl::PointXYZ(-x/2 ,-y/2 ,-z/2 ) );
}

void RobotModel::createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ> &cylinder_cloud, double radius, double height, int number_of_step_alpha )
{
    double alpha_angle=M_PI/number_of_step_alpha;
    double z=height/2;
    double x,y;
    for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
    {
        x=radius*cos(alpha);
        y=radius*sin(alpha);
        cylinder_cloud.push_back(pcl::PointXYZ(x,y,z));
        cylinder_cloud.push_back(pcl::PointXYZ(x,y,-z));
    }
}

void RobotModel::createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ> &sphere_cloud, double radius, int number_of_step_alpha, int number_of_step_beta)
{
    // alpha cretae point on a circle given radius, beta will give you the radius,
    double alpha_angle, beta_angle;
    alpha_angle= M_PI/number_of_step_alpha;
    beta_angle= M_PI/number_of_step_beta;
    double x,y,z,new_radius;
    for(double beta=0;beta<M_PI/2;beta=beta+beta_angle)
    {
        new_radius=radius*cos(beta);
        z=radius*sin(beta);
        for(double alpha=0;alpha<2*M_PI;alpha=alpha+alpha_angle)
        {
            x=new_radius*cos(alpha);
            y=new_radius*sin(alpha);
            sphere_cloud.push_back(pcl::PointXYZ(x,y,z));
            sphere_cloud.push_back(pcl::PointXYZ(x,y,-z));
        }
    }
    //top and bottom of the sphere
    sphere_cloud.push_back(pcl::PointXYZ(0,0,+radius));
    sphere_cloud.push_back(pcl::PointXYZ(0,0,-radius));
}

void RobotModel::subtractingPtCloudsFullBody(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_2, 
                                             pcl::PointCloud<pcl::PointXYZ>::Ptr subtracted_cloud)
{
    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    // Set alpha, which is the maximum length from a vertex to the center of the voronoi cell (the smaller, the greater the resolution of the hull).
    //    http://robotica.unileon.es/mediawiki/index.php/PCL/OpenNI_tutorial_3:_Cloud_processing_(advanced)#Concave_hull
    //
    //pcl::ConcaveHull<pcl::PointXYZ> hull;
    //hull.setAlpha(0.1);
    
    hull.setInputCloud(cloud_2);
    hull.setDimension(3);
    hull.reconstruct(*boundingbox_ptr.get(),polygons);

    std::vector<int> indices;
    pcl::CropHull<pcl::PointXYZ> bb_filter;

    bb_filter.setDim(3);
    bb_filter.setInputCloud(cloud_1);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(boundingbox_ptr);
    bb_filter.filter(indices);

    pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
    fInliers->indices=indices ;
    pcl::ExtractIndices<pcl::PointXYZ> extract ;
    extract.setInputCloud (cloud_1);
    extract.setIndices (fInliers);
    extract.setNegative (true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*output_cloud);    

    pclStatisticalOutlierRemoval(output_cloud, subtracted_cloud);
}

void RobotModel::subtractingPtClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr env_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr robot_link_cloud)
{

    std::vector<pcl::Vertices> polygons;
    pcl::PointCloud<pcl::PointXYZ>::Ptr boundingbox_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConvexHull<pcl::PointXYZ> hull;

    hull.setInputCloud(robot_link_cloud);
    hull.setDimension(3);
    hull.reconstruct(*boundingbox_ptr.get(),polygons);

    std::vector<int> indices;
    pcl::CropHull<pcl::PointXYZ> bb_filter;

    bb_filter.setDim(3);
    bb_filter.setInputCloud(env_cloud);
    bb_filter.setHullIndices(polygons);
    bb_filter.setHullCloud(boundingbox_ptr);
    bb_filter.filter(indices);

    pcl::PointIndices::Ptr fInliers (new pcl::PointIndices);
    fInliers->indices=indices ;
    pcl::ExtractIndices<pcl::PointXYZ> extract ;
    extract.setInputCloud (env_cloud);
    extract.setIndices (fInliers);
    extract.setNegative (true);    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    extract.filter (*env_cloud);       
}
void RobotModel::selfFilterFullbody(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr, 
                                    std::string sensor_frame_name, USESELFCOLLISION use_selfcollision)
{    
    double start_time = omp_get_wtime();

    RobotState robot_state= this->getRobotState();
    std::string A_Frame_Name=sensor_frame_name;
    std::string B_Frame_Name=this->getURDF()->getRoot()->name ;

    pcl::PointCloud<pcl::PointXYZ> link_pointcloud_transformed_in_sensor_frame, robot_pointcloud;

    for(std::map<std::string, RobotLink>::iterator it=robot_state.robot_links_.begin();it!=robot_state.robot_links_.end();it++)
    {
        KDL::Frame link_pose_in_sensor_frame;
        Eigen::Isometry3f link_pose_in_sensor_frame_eigen_matrix;
        if(use_selfcollision == motion_planners::VISUAL)
        {
            std::vector<urdf::VisualSharedPtr > links_collision=it->second.getLinkVisuals();
            std::vector<pcl::PointCloud<pcl::PointXYZ> > collision_point_cloud=it->second.getVisualPointCloud();
            for(std::size_t i=0;i<links_collision.size();i++)
            {
                KDL::Frame link_pose_in_base_link=toKdl( links_collision.at(i)->origin);
                this->ConvertPoseBetweenFrames( B_Frame_Name, link_pose_in_base_link , A_Frame_Name , link_pose_in_sensor_frame);
                KDLFrameToEigenMatrix(link_pose_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                pcl::transformPointCloud (collision_point_cloud.at(i), link_pointcloud_transformed_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                robot_pointcloud = robot_pointcloud + link_pointcloud_transformed_in_sensor_frame;
            }
        }
        else
        {
            std::vector<urdf::CollisionSharedPtr > links_collision=it->second.getLinkCollisions();
            std::vector<pcl::PointCloud<pcl::PointXYZ> > collision_point_cloud=it->second.getCollisionPointCloud();
            for(std::size_t i=0;i<links_collision.size();i++)
            {
                KDL::Frame link_pose_in_base_link=toKdl( links_collision.at(i)->origin);
                this->ConvertPoseBetweenFrames( B_Frame_Name, link_pose_in_base_link , A_Frame_Name , link_pose_in_sensor_frame);
                KDLFrameToEigenMatrix(link_pose_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                pcl::transformPointCloud (collision_point_cloud.at(i), link_pointcloud_transformed_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                robot_pointcloud = robot_pointcloud + link_pointcloud_transformed_in_sensor_frame;
            }
        }
    }
    // pcl::io::savePCDFile("robot.pcd",robot_pointcloud);    
    double end_time = omp_get_wtime();
    LOG_DEBUG_S<<"[selfFilter:] Transforming point cloud took "<< end_time - start_time;
    //std::cout<<"  Size = "<<scene_ptr->size()<<"  "<<robot_pointcloud.size()<<"   "<<std::endl;
    start_time = omp_get_wtime();
    subtractingPtCloudsFullBody( scene_ptr, boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(robot_pointcloud ) , new_scene_ptr);    
    end_time = omp_get_wtime();

    LOG_DEBUG_S<<"[selfFilter:] Subtracting point cloud took "<< end_time - start_time;

    return;
}

void RobotModel::selfFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ptr, std::string sensor_frame_name, USESELFCOLLISION use_selfcollision)
{ 
    double start_time = omp_get_wtime();

    RobotState robot_state= this->getRobotState();
    std::string A_Frame_Name=sensor_frame_name;
    std::string B_Frame_Name=this->getURDF()->getRoot()->name ;

    pcl::PointCloud<pcl::PointXYZ> link_pointcloud_transformed_in_sensor_frame;
 
    for(std::map<std::string, RobotLink>::iterator it=robot_state.robot_links_.begin();it!=robot_state.robot_links_.end();it++)
    {
        KDL::Frame link_pose_in_sensor_frame;
        Eigen::Isometry3f link_pose_in_sensor_frame_eigen_matrix;
        if(use_selfcollision == motion_planners::VISUAL)
        {
            std::vector<urdf::VisualSharedPtr > links_collision=it->second.getLinkVisuals();
            std::vector<pcl::PointCloud<pcl::PointXYZ> > collision_point_cloud=it->second.getVisualPointCloud();
            for(std::size_t i=0;i<links_collision.size();i++)
            {
                KDL::Frame link_pose_in_base_link=toKdl( links_collision.at(i)->origin);
                this->ConvertPoseBetweenFrames( B_Frame_Name, link_pose_in_base_link , A_Frame_Name , link_pose_in_sensor_frame);
                KDLFrameToEigenMatrix(link_pose_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                pcl::transformPointCloud (it->second.getVisualPointCloud().at(i), link_pointcloud_transformed_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                subtractingPtClouds(scene_ptr, boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(link_pointcloud_transformed_in_sensor_frame) );
            }
        }
        else
        {
            std::vector<urdf::CollisionSharedPtr > links_collision=it->second.getLinkCollisions();
            std::vector<pcl::PointCloud<pcl::PointXYZ> > collision_point_cloud=it->second.getCollisionPointCloud();
            for(std::size_t i=0;i<links_collision.size();i++)
            {
                KDL::Frame link_pose_in_base_link=toKdl( links_collision.at(i)->origin);
                this->ConvertPoseBetweenFrames( B_Frame_Name, link_pose_in_base_link , A_Frame_Name , link_pose_in_sensor_frame);
                KDLFrameToEigenMatrix(link_pose_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);
                pcl::transformPointCloud (collision_point_cloud.at(i), link_pointcloud_transformed_in_sensor_frame, link_pose_in_sensor_frame_eigen_matrix);

                subtractingPtClouds(scene_ptr, boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >(link_pointcloud_transformed_in_sensor_frame) );
            }
        }
    }
    // pcl::io::savePCDFile("robot.pcd",robot_pointcloud);    
    double end_time = omp_get_wtime();
    LOG_DEBUG_S<<"[selfFilter:] Subtracting point cloud took "<< end_time - start_time;
    return;    
}

void RobotModel::pclStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (input_cloud);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*filtered_cloud);
}

*/

