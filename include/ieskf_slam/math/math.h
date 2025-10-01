#pragma once
#include <Eigen/Dense>
#include <numeric>
namespace IESKFSlam {
static Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d ans;
    ans.setIdentity();
    ans.block<3, 3>(0, 0) = q.toRotationMatrix();
    ans.block<3, 1>(0, 3) = t;
    return ans;
}
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& data, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = data.size();
    assert(len > 1);
    // clang-format off
    mean = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) / len;
    cov_diag = std::accumulate(data.begin(), data.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) / (len - 1);
    // clang-format on
}
}  // namespace IESKFSlam