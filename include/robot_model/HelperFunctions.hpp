#ifndef HELPERFUNCTIONS_HPP
#define HELPERFUNCTIONS_HPP

#include <vector>

#include <kdl/frames.hpp>
#include <urdf_parser/urdf_parser.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace robot_model
{

    KDL::Vector toKdl(const urdf::Vector3 &v);
    KDL::Rotation toKdl(const urdf::Rotation &r);
    KDL::Frame toKdl(const urdf::Pose &p);
    urdf::Pose toURDFPose(const KDL::Frame &frame);
    void KDLFrameToEigenMatrix(const KDL::Frame &frame, Eigen::Isometry3f &transform);
}

#endif
