
#include "robot_model/HelperFunctions.hpp"

namespace robot_model
{

    KDL::Vector toKdl(const urdf::Vector3 &v)
    {
        return KDL::Vector(v.x, v.y, v.z);
    }

    // construct rotation
    KDL::Rotation toKdl(const urdf::Rotation &r)
    {
        return KDL::Rotation::Quaternion(r.x, r.y, r.z, r.w);
    }

    // construct pose
    KDL::Frame toKdl(const urdf::Pose &p)
    {
        return KDL::Frame(toKdl(p.rotation), toKdl(p.position));
    }

    urdf::Pose toURDFPose(const KDL::Frame &frame)
    {
        urdf::Pose urdf_pose;
        urdf_pose.position.x = frame.p.x();
        urdf_pose.position.y = frame.p.y();
        urdf_pose.position.z = frame.p.z();

        frame.M.GetQuaternion(
            urdf_pose.rotation.x,
            urdf_pose.rotation.y,
            urdf_pose.rotation.z,
            urdf_pose.rotation.w);

        return urdf_pose;
    }

    void KDLFrameToEigenMatrix(KDL::Frame &frame, Eigen::Isometry3f &transform)
    {
        transform.setIdentity(); // ensures last row is [0 0 0 1]
        transform.translation() << frame.p.x(), frame.p.y(), frame.p.z();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                transform(i, j) = frame.M(i, j);
            }
        }
    }

}
