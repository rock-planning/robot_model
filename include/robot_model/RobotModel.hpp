#ifndef ROBOTMODEL_HPP
#define ROBOTMODEL_HPP

#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv_givens.hpp>
#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/joint.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <urdf_model/types.h>
#include <urdf_model/link.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>


#include "robot_model/RobotModelConfig.hpp"
#include "robot_model/RobotLink.hpp"
#include "robot_model/RobotJoint.hpp"

#include <base-logging/Logging.hpp>
#include <base/Pose.hpp>
#include <base/samples/Joints.hpp>
#include <base/JointLimits.hpp>

#include <collision_detection/abstract/AbstractCollisionDetection.hpp>
#include <kinematics_library/KinematicsFactory.hpp>

#include <chrono>  // for high_resolution_clock - Available for C+11


namespace robot_model
{
    
template<typename to, typename from>
to lexical_cast(from const &x)
{
    std::stringstream os;
    to ret;
    os << x;
    os >> ret;
    return ret;
}

class RobotState
{
    public:
        RobotState(){};
        std::map<std::string, RobotLink> robot_links_;
        std::map<std::string, RobotJoint> robot_joints_;
};

class RobotModel
{
    public:
        RobotModel(RobotModelConfig robot_model_config, double link_padding = 1.00);

        bool initialization();    

        bool getPlanningGroupJointInformation(  const std::string  planning_group_name, std::vector< std::pair<std::string,urdf::Joint> > &planning_groups_joints,
                                                std::vector< std::string> &planning_group_joints_name);

        bool getPlanningGroupJointInformation(const std::string planning_group_name, std::vector< std::pair<std::string,urdf::Joint> > &planning_groups_joints) const;
        
        bool getPlanningGroupJointInformation(const std::string planning_group_name, base::samples::Joints &planning_groups_joints);

        void getPlanningGroupJointsName(const std::string planning_group_name, std::vector< std::string> &planning_group_joints_name);
        
        bool getJointsInformation(const std::string &base_link, const std::string &tip_link, base::samples::Joints &joints) const;

        base::samples::Joints getRobotJointsState();
        
        // outputs actual collision object name with its radius. If no radius is available then it will output -1 as radius.
        void getPlanningGroupCollisionObjectsNameWithRadius(const std::string planning_group_name, 
                                                            std::vector< std::pair<std::string, double>> &planning_group_collision_link_names);

        void setSRDF(boost::shared_ptr<srdf::Model> &srdf_model_);

        void setURDF(urdf::ModelInterfaceSharedPtr &urdf_model_);

        boost::shared_ptr<srdf::Model>  const & getSRDF();

        urdf::ModelInterfaceSharedPtr const &  getURDF();

        inline RobotState getRobotState () const { return robot_state_;}

        inline void setRobotState(RobotState &robot_state ){robot_state_ = robot_state;}

        void updateJoint(std::string joint_name, double joint_value);

        void updateJointGroup(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values);

        void updateJointGroup(const base::samples::Joints &joint_values);

        void updateJointGroup(const std::map< std::string ,double > &joint_values);

        void updateJointGroup(const std::vector<std::string> &joint_names,const std::vector<double> &joint_values);

        void updateOctomap(const std::shared_ptr<octomap::OcTree> &octomap, std::string collision_object_name="");

        void assignPlanningScene( const std::shared_ptr<octomap::OcTree> &octomap, const std::string &link_name, std::string collision_object_name);
        
        void updateOctomapAsBoxes(const std::shared_ptr<octomap::OcTree> &octomap, std::string collision_object_name);
        
        void assignPlanningSceneAsBoxes(   const std::shared_ptr<octomap::OcTree> &octomap, const std::string &link_name, std::string collision_object_name);

        bool isStateValid(double &collision_cost);

        void convertPoseBetweenFrames( const std::string B_Frame_Name, const KDL::Frame &F_B_C , const std::string &A_Frame_Name ,KDL::Frame &F_A_C );

        void getRobotCollisions(std::vector<urdf::CollisionSharedPtr > &  robotCollisions);

        void getRobotVisuals(std::vector<urdf::VisualSharedPtr > &  robotVisuals);

        void addCollisionsToWorld(urdf::CollisionSharedPtr &  robotCollision, std::string link_name);

        void generateRandomJointValue(const std::string  &planning_group_name, std::map<std::string, double>   &planning_groups_joints_with_random_values);

        float randomFloat(const float& min,const  float &max);

        boost::filesystem::path resolve_path( const boost::filesystem::path& p, const boost::filesystem::path& base = boost::filesystem::current_path());

        std::string getURDFFileAbsolutePath();  

        inline void setRobotCollisionDetector(collision_detection::AbstractCollisionPtr collision_detector){robot_collision_detector_ = collision_detector;}

        void setWorldCollisionDetector(collision_detection::AbstractCollisionPtr collision_detector);

        void setDisabledEnvironmentCollision(std::vector <std::pair<std::string,std::string> > disabled_collision_pair);

        inline void setKinematicsSolver(std::string name, kinematics_library::AbstractKinematicPtr robot_kinematics){robot_kinematics_map_[name] = robot_kinematics;}

        std::vector< collision_detection::DistanceInformation>& getSelfDistanceInfo();

        void computeJacobain(const std::string &chain_root_link,const  std::string& tip_link, std::map<std::string, double> joints_name_values, KDL::Jacobian  &jacobian);

        void manipulabilityIndex(KDL::Jacobian  &jacobian, double &manipulability_index);

        void addGraspObject(urdf::CollisionSharedPtr grasp_object, std::string parent_link_name);

        bool removeGraspObject(const std::string grasp_object_name);

        bool removeWorldObject(const std::string world_object_name);
        
        inline bool removeObjectFromOctree(Eigen::Vector3d object_pose, Eigen::Vector3d object_size)
        {
            return world_collision_detector_->removeObjectFromOctree(object_pose, object_size);
        }

        inline void saveOctree()
        {
            return world_collision_detector_->saveOctree();
        }
        
        void printWorldCollisionObject();

        std::vector< std::pair<std::string, std::string> > getCollidedObjectsNames()
        {
            return robot_collision_detector_->getCollidedObjectsNames();
        }

        void setDefaultJointWeight(const std::vector< std::string > &planning_joints_name);
        
        int& robot_state();

        inline void setPlanningGroupName(std::string planning_group_name){planning_group_name_ = planning_group_name;}

        std::string getPlanningGroupName(){return planning_group_name_;}

        bool getJointLimits(std::vector< double > &lower_limits, std::vector< double > &upper_limits);

        bool getJointLimits(base::JointLimits &limits);

        std::map<std::string, kinematics_library::AbstractKinematicPtr> robot_kinematics_map_;
        
        bool getChainLinksName(std::string base_link, std::string tip_link, std::vector< std::string> &planning_group_link_name);

        void getLinkTransformByName(const std::string link_name, Eigen::Vector3d &position, Eigen::Vector4d &orientation);

        bool getChainJointState(std::string base_link, std::string tip_link, std::map<std::string, double> &planning_groups_joints);

        bool getRobotCollisionInfo(std::vector<collision_detection::DistanceInformation> &contact_info);
       
        std::vector< collision_detection::DistanceInformation > getRobotCollisionDistanceInformation()
        {
            return robot_collision_detector_->getCollisionDistanceInformation();
        }
        
        std::vector< collision_detection::DistanceInformation > getRobotCompleteDistanceInformation()
        {
            return robot_collision_detector_->getCompleteDistanceInformation();
        }

        bool getKinematicsSolver(const std::string &name, kinematics_library::AbstractKinematicPtr &kinematic_solver);
        
        const collision_detection::AbstractCollisionPtr& getRobotCollisionDetector(){return robot_collision_detector_;}
        
        std::string getWorldFrameName(){return world_frame_;}
        
        std::string getBaseFrameName(){return base_frame_;}
        
        std::string getTipFrameName(){return tip_frame_;}

        bool getBaseAndTipFramesNames(const std::string &planning_group_name, std::string &base_name, std::string &tip_name);

    private :

        RobotState  robot_state_;
        std::string planning_group_name_;   
        KDL::Tree kdl_tree_;
        double link_padding_;
        KDL::Chain kdl_chain_;
        boost::shared_ptr<srdf::Model> srdf_model_;
        urdf::ModelInterfaceSharedPtr urdf_model_;
        std::string urdf_file_abs_path_;
        std::string srdf_file_abs_path_;
        std::string world_frame_, base_frame_, tip_frame_;
        collision_detection::AbstractCollisionPtr robot_collision_detector_, world_collision_detector_;

        bool initialiseURDFandSRDF();
        
        void dfsTraversing(std::string start_link_name, std::vector<std::string>& visited_links);

        void settingVisitedFlagLinkToFalse();

        void settingVisitedFlagLinkToFalse(std::vector<std::string> visted_links_names);
        
        bool initializeLinksCollisions();

        void kdlFrameToEigenMatrix(KDL::Frame &frame,Eigen::Isometry3f &transform);
        
        bool getPlanningGroup(const std::string &planning_group_name,  std::string &base_link, std::string &tip_link, 
                              KDL::Chain &kdl_chain) const;

};

using RobotModelPtr = std::shared_ptr<robot_model::RobotModel>;

}// end namespace motion_planners
#endif // ROBOTMODEL_HPP

/* Subtracting the pointcloud using this method is not efficient because it creates a convex hull for the entire robot.
 * In doing so, computation time is less but it will delete all the points between the robot links.
 *
 * The commented functions are not used due to the reason mentioned above and also self-filter should be done outside the motion planner
 * Once in a while we also had issues with PCL and c++11. So due to the reasons mentioned above, functions related to plc based self filter are commented.
 */
/*
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

template<class urdfT> 
void registerLinks(const urdfT &urdf_link, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud );
void scalePointCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out,
                      double scale_x, double scale_y, double scale_z);
void createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,double x, double y, double z, 
                          Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix, bool dense);
void createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ> &box_cloud,double x, double y, double z);
void createPtCloudFromCylinder( pcl::PointCloud< pcl::PointXYZ >::Ptr transformed_cylinder_cloud_ptr, double radius, double height, 
                                Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha = 10, bool dense = false );
void createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ> &cylinder_cloud, double radius, double height, int number_of_step_alpha=5);
void createPtCloudFromSphere(   pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_sphere_cloud_ptr, double radius, 
                                Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix, 
                                int number_of_step_alpha=10, int number_of_step_beta=10);
void createPtCloudFromSphere(pcl::PointCloud<pcl::PointXYZ> &sphere_cloud, double radius, int number_of_step_alpha=20, int number_of_step_beta=20);
void createPtCloudFromBox(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_box_cloud_ptr,double x, double y, double z,
                            Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix);
void createPtCloudFromCylinder(pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cylinder_cloud_ptr, double radius, double height,
                                Eigen::Isometry3f link_visual_pose_in_sensor_frame_eigen_matrix, int number_of_step_alpha=10 );
void createPointCloudFromVisual(std::vector<urdf::VisualSharedPtr > &link_visuals, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud );
void createPointCloudFromCollision(std::vector<urdf::CollisionSharedPtr> &link_collisions, std::vector<pcl::PointCloud<pcl::PointXYZ> > &link_point_cloud );
void subtractingPtCloudsFullBody(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud_2, 
                                pcl::PointCloud<pcl::PointXYZ>::Ptr subtracted_cloud);
void subtractingPtClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr env_cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr robot_link_cloud);
void selfFilterFullbody(pcl::PointCloud<pcl::PointXYZ>::ConstPtr scene_ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr new_scene_ptr, 
                        std::string sensor_frame_name, USESELFCOLLISION use_selfcollision);
void selfFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr scene_ptr, std::string sensor_frame_name, USESELFCOLLISION use_selfcollision);
void pclStatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud);*/
