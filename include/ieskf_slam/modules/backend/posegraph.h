#pragma once
#include "ieskf_slam/type/pose.h"
#include <gtsam/geometry/Pose3.h>

namespace IESKFSlam {
    class PoseGraphOpt {
    private:
        gtsam::Pose3 eigenPoseToGtsam(const Pose& eigen_pose);
        void gtsamPoseToEigen(const gtsam::Pose3& gtsam_pose, Pose& eigen_pose);
        
    public:
        PoseGraphOpt();
        bool slove(std::vector<Pose> &poses, std::vector<BinaryEdge> &bes);
    };
} // namespace IESKFSlam