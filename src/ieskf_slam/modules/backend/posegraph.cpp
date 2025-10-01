// #include "ieskf_slam/modules/pose_graph_opt/pose_graph_opt.h"
#include "ieskf_slam/modules/backend/posegraph.h"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

namespace IESKFSlam {

PoseGraphOpt::PoseGraphOpt(/* args */) {}

bool PoseGraphOpt::slove(std::vector<Pose>& poses, std::vector<BinaryEdge>& bes) {
    if (poses.size() <= 10)
        return false;

    // 1. 创建因子图
    gtsam::NonlinearFactorGraph graph;

    // 2. 创建初始值
    gtsam::Values initial;

    // 3. 添加先验因子（固定第一帧）
    gtsam::noiseModel::Diagonal::shared_ptr priorNoise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

    // 将第一帧姿态转换为gtsam::Pose3
    gtsam::Pose3 first_pose = eigenPoseToGtsam(poses[0]);
    graph.addPrior(gtsam::symbol('x', 0), first_pose, priorNoise);
    initial.insert(gtsam::symbol('x', 0), first_pose);

    // 4. 添加帧间约束（里程计因子）
    gtsam::noiseModel::Diagonal::shared_ptr odometryNoise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());

    for (int i = 0; i < poses.size() - 1; i++) {
        // 计算相对位姿：T_i_to_i+1 = T_i^{-1} * T_i+1
        gtsam::Pose3 relative_pose = eigenPoseToGtsam(poses[i]).inverse() * eigenPoseToGtsam(poses[i + 1]);

        // 添加帧间因子
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol('x', i), gtsam::symbol('x', i + 1), relative_pose,
                                                     odometryNoise));

        // 添加初始值
        if (i > 0) {  // 第一帧已经添加过了
            initial.insert(gtsam::symbol('x', i), eigenPoseToGtsam(poses[i]));
        }
    }
    // 添加最后一帧的初始值
    initial.insert(gtsam::symbol('x', poses.size() - 1), eigenPoseToGtsam(poses.back()));

    // 5. 添加回环约束
    gtsam::noiseModel::Diagonal::shared_ptr loopNoise =
        gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished());

    for (auto&& be : bes) {
        gtsam::Pose3 relative_pose = eigenPoseToGtsam(be.constraint);

        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam::symbol('x', be.to_vertex),
                                                     gtsam::symbol('x', be.from_vertex), relative_pose, loopNoise));
    }

    // 6. 优化
    gtsam::LevenbergMarquardtParams params;
    params.maxIterations = 200;
    params.relativeErrorTol = 1e-5;
    params.absoluteErrorTol = 1e-5;

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values result = optimizer.optimize();

    // 7. 输出优化信息
    std::cout << "Initial error: " << graph.error(initial) << std::endl;
    std::cout << "Final error: " << graph.error(result) << std::endl;
    std::cout << "Iterations: " << optimizer.iterations() << std::endl;

    // 8. 更新优化后的位姿
    for (size_t i = 0; i < poses.size(); i++) {
        gtsam::Pose3 optimized_pose = result.at<gtsam::Pose3>(gtsam::symbol('x', i));
        gtsamPoseToEigen(optimized_pose, poses[i]);
    }

    return true;
}

// 辅助函数：将Eigen位姿转换为GTSAM Pose3
gtsam::Pose3 PoseGraphOpt::eigenPoseToGtsam(const Pose& eigen_pose) {
    gtsam::Rot3 rotation(gtsam::Quaternion(eigen_pose.rotation.w(), eigen_pose.rotation.x(), eigen_pose.rotation.y(),
                                           eigen_pose.rotation.z()));
    gtsam::Point3 translation(eigen_pose.position.x(), eigen_pose.position.y(), eigen_pose.position.z());
    return gtsam::Pose3(rotation, translation);
}

// 辅助函数：将GTSAM Pose3转换为Eigen位姿
void PoseGraphOpt::gtsamPoseToEigen(const gtsam::Pose3& gtsam_pose, Pose& eigen_pose) {
    gtsam::Quaternion quat = gtsam_pose.rotation().toQuaternion();
    eigen_pose.rotation = Eigen::Quaterniond(quat.w(), quat.x(), quat.y(), quat.z());
    eigen_pose.position = Eigen::Vector3d(gtsam_pose.x(), gtsam_pose.y(), gtsam_pose.z());
}

}  // namespace IESKFSlam