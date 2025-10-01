#pragma once
#include <Eigen/Dense>
namespace IESKFSlam {
static Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d ans;
    ans.setIdentity();
    ans.block<3, 3>(0, 0) = q.toRotationMatrix();
    ans.block<3, 1>(0, 3) = t;
    return ans;
}
}  // namespace IESKFSlam