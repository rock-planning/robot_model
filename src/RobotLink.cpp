#include "robot_model/RobotLink.hpp"
#include <boost/make_shared.hpp>

using namespace robot_model;

RobotLink::RobotLink()
{
    link_collisions_names_.clear();
    link_collisions_names_with_radius_.clear();
}

std::vector<urdf::Pose> &RobotLink::getLinkCollisionRelativePose()
{
    return link_collision_relative_pose_;
}

std::vector<urdf::Pose> &RobotLink::getLinkVisualRelativePose()
{
    return link_visual_relative_pose_;
}

void RobotLink::calculateLinkVisualsPoseInGlobalPose()
{
    for (std::size_t i = 0; i < link_visuals_.size(); ++i)
    {
        const auto global_pose = getLinkFrame() * toKdl(link_visual_relative_pose_[i]);
        global_pose.M.GetQuaternion(
            link_visuals_[i].origin.rotation.x,
            link_visuals_[i].origin.rotation.y,
            link_visuals_[i].origin.rotation.z,
            link_visuals_[i].origin.rotation.w);
        link_visuals_[i].origin.position.x = global_pose.p.x();
        link_visuals_[i].origin.position.y = global_pose.p.y();
        link_visuals_[i].origin.position.z = global_pose.p.z();
    }
}

void RobotLink::calculateLinkCollisionPoseinGlobalPose()
{
    for (std::size_t i = 0; i < link_collisions_.size(); ++i)
    {
        const auto global_pose = getLinkFrame() * toKdl(link_collision_relative_pose_[i]);
        global_pose.M.GetQuaternion(
            link_collisions_[i].origin.rotation.x,
            link_collisions_[i].origin.rotation.y,
            link_collisions_[i].origin.rotation.z,
            link_collisions_[i].origin.rotation.w);
        link_collisions_[i].origin.position.x = global_pose.p.x();
        link_collisions_[i].origin.position.y = global_pose.p.y();
        link_collisions_[i].origin.position.z = global_pose.p.z();
    }
}

std::vector<urdf::VisualSharedPtr> RobotLink::getLinkVisuals()
{
    std::vector<urdf::VisualSharedPtr> result;
    result.reserve(link_visuals_.size());
    for (const auto &visual : link_visuals_)
    {
        result.emplace_back(boost::make_shared<urdf::Visual>(visual));
    }
    return result;
}

void RobotLink::setLinkVisuals(std::vector<urdf::VisualSharedPtr> &links)
{
    for (const auto &link : links)
    {
        link_visuals_.emplace_back(*link);
        link_visual_relative_pose_.emplace_back(link->origin);
    }
}

void RobotLink::setLinkCollisions(std::vector<urdf::CollisionSharedPtr> &links)
{
    for (const auto &link : links)
    {
        link_collisions_.emplace_back(*link);
        link_collision_relative_pose_.emplace_back(link->origin);
    }
}

void RobotLink::setLinkCollision(const urdf::CollisionSharedPtr &link_collision)
{
    link_collisions_.emplace_back(*link_collision);
    link_collision_relative_pose_.emplace_back(link_collision->origin);
}

void RobotLink::getLinkVisuals(std::vector<urdf::VisualSharedPtr> &link_visuals) // TODO
{
    for (const auto &i : link_visuals_)
    {
        link_visuals.emplace_back(boost::make_shared<urdf::Collision>(i));
    }
}

std::vector<urdf::CollisionSharedPtr> RobotLink::getLinkCollisions()
{
    // return this->link_collisions;
    std::vector<urdf::CollisionSharedPtr> result;
    result.reserve(link_collisions_.size());
    for (const auto &col : link_collisions_)
    {
        result.emplace_back(boost::make_shared<urdf::Collision>(col));
    }
    return result;
}

void RobotLink::getLinkCollisions(std::vector<urdf::CollisionSharedPtr> &link_collision)
{
    link_collision.reserve(link_collision.size() + link_collisions_.size());
    for (const auto &col : link_collisions_)
    {
        link_collision.emplace_back(boost::make_shared<urdf::Collision>(col));
    }
}

std::string &RobotLink::getLinkName()
{
    return link_name_;
}

void RobotLink::setLinkDFSVisited(bool visited)
{
    dfs_visited_ = visited;
}

bool RobotLink::getLinkDFSVisited()
{
    return dfs_visited_;
}

void RobotLink::setLinkName(const std::string &link_name)
{
    link_name_ = link_name;
}

void RobotLink::setLinkFrame(const KDL::Frame &link_frame)
{
    link_frame_ = link_frame;
}

void RobotLink::setLinkCollisionsNameWithRadius(const std::string &collision_object_name, double radius)
{
    link_collisions_names_.emplace_back(collision_object_name);
    link_collisions_names_with_radius_.emplace_back(collision_object_name, radius);
}

KDL::Frame RobotLink::getLinkFrame()
{
    return link_frame_;
}

void RobotLink::AddCollision(urdf::CollisionSharedPtr collision)
{
    link_collision_relative_pose_.emplace_back(collision->origin);

    const auto global_pose = getLinkFrame() * toKdl(collision->origin);
    global_pose.M.GetQuaternion(
        collision->origin.rotation.x,
        collision->origin.rotation.y,
        collision->origin.rotation.z,
        collision->origin.rotation.w);
    collision->origin.position.x = global_pose.p.x();
    collision->origin.position.y = global_pose.p.y();
    collision->origin.position.z = global_pose.p.z();

    link_collisions_.emplace_back(*collision);
}

/* Subtracting the pointcloud using this method is not efficient because it creates a convex hull for the entire robot.
 * In doing so, computation time is less but it will delete all the points between the robot links.
 *
 * The commented functions are not used due to the reason mentioned above and also self-filter should be done outside the motion planner
 * Once in a while we also had issues with PCL and c++11. So due to the reasons mentioned above, functions related to plc based self filter are commented.
 */
/*
void RobotLink::setVisualPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds)
{
    link_visual_point_clouds_ = link_point_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZ> > RobotLink::getVisualPointCloud()
{
    return link_visual_point_clouds_;
}


void RobotLink::setCollisionPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds)
{
    link_collision_point_clouds_ = link_point_clouds;
}

std::vector<pcl::PointCloud<pcl::PointXYZ> >& RobotLink::getCollisionPointCloud()
{
    return link_collision_point_clouds_;
}*/
