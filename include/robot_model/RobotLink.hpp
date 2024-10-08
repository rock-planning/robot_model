#ifndef ROBOTLINK_HPP
#define ROBOTLINK_HPP

#include <vector>
#include <string>

#include <urdf_model/model.h>
#include <kdl/frames.hpp>
#include <boost/shared_ptr.hpp>
#include "HelperFunctions.hpp"

namespace robot_model
{

class RobotLink
{

    public:
        RobotLink();
        std::vector<urdf::Pose> & getLinkCollisionRelativePose();
        std::vector<urdf::Pose> & getLinkVisualRelativePose();
        void calculateLinkVisualsPoseInGlobalPose();
        void calculateLinkCollisionPoseinGlobalPose();
        std::vector<urdf::VisualSharedPtr >  getLinkVisuals();
        void setLinkVisuals( std::vector<urdf::VisualSharedPtr > &link_visuals);
        void setLinkCollisions( std::vector<urdf::CollisionSharedPtr > &link_collisions);
        // set the collision object name as registered to a collision library
        void setLinkCollisionsNameWithRadius( std::string collision_object_name, double radius);
        void setLinkCollision( const urdf::CollisionSharedPtr &link_collision);
        void getLinkVisuals(std::vector<urdf::VisualSharedPtr > &link_visuals );
        void getLinkCollisions(std::vector<urdf::CollisionSharedPtr > &link_collision);
        const std::vector<std::string>& getLinkCollisionsNames(){return link_collisions_names_;};
        const std::vector<std::pair<std::string, double>>& getLinkCollisionsNamesWithRadius(){return link_collisions_names_with_radius_;};
        std::vector<urdf::CollisionSharedPtr >  getLinkCollisions();
        std::string &getLinkName();
        void setLinkDFSVisited(bool visited);
        bool getLinkDFSVisited();
        void setLinkName(std::string &link_name);
        void setLinkFrame(KDL::Frame &link_frame);
        KDL::Frame getLinkFrame();
        void AddCollision(urdf::CollisionSharedPtr collision);
        void clearLinkCollisionsNames(){link_collisions_names_.clear();}

    private:
        bool dfs_visited_;
        std::string link_name_;
        KDL::Frame link_frame_;
        std::vector<urdf::Visual > link_visuals_;
        std::vector<urdf::Collision> link_collisions_;        
        std::vector<urdf::Pose> link_visual_relative_pose_, link_collision_relative_pose_;
        std::vector<std::string> link_collisions_names_;
        std::vector<std::pair<std::string, double>> link_collisions_names_with_radius_;

};
}// end namespace motion_planners
#endif // ROBOTLINK_HPP


/* Subtracting the pointcloud using this method is not efficient because it creates a convex hull for the entire robot.
 * In doing so, computation time is less but it will delete all the points between the robot links.
 *
 * The commented functions are not used due to the reason mentioned above and also self-filter should be done outside the motion planner
 * Once in a while we also had issues with PCL and c++11. So due to the reasons mentioned above, functions related to plc based self filter are commented.
 */
/*
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
std::vector<pcl::PointCloud<pcl::PointXYZ> > link_visual_point_clouds_, link_collision_point_clouds_;
void setCollisionPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds);
std::vector<pcl::PointCloud<pcl::PointXYZ> >& getCollisionPointCloud();
void setVisualPointCloud(std::vector<pcl::PointCloud<pcl::PointXYZ> > link_point_clouds);
std::vector<pcl::PointCloud<pcl::PointXYZ> > getVisualPointCloud();*/










