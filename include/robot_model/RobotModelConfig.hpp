#ifndef ROBOTMODEL_CONFIG_HPP
#define ROBOTMODEL_CONFIG_HPP

#include <string>
#include <vector>


namespace robot_model
{


enum USESELFCOLLISION
{
    VISUAL, COLLISION
};


/**
 *  @struct RobotModelParameters.
 * @brief This struct contains parameters used in the robot model.
 */
struct RobotModelConfig
{
    // srdf file abs path
    std::string srdf_file;
    // urdf file abs path
    std::string urdf_file;
    // planning group
    std::string planning_group_name;
};

}
#endif // ROBOTMODEL_CONFIG_HPP