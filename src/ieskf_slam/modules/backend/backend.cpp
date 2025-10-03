#pragma once
#define PCL_NO_PRECOMPILE
#include "ieskf_slam/modules/backend/backend.h"

#include "ieskf_slam/globaldefine.h"
#include <open3d/pipelines/registration/Registration.h>
#include <open3d/pipelines/registration/TransformationEstimation.h>
#include <open3d/geometry/PointCloud.h>

namespace IESKFSlam {

// 辅助函数：PCL点云转Open3D点云
std::shared_ptr<open3d::geometry::PointCloud> pclToOpen3d(const PCLPointCloud& pcl_cloud) {
    auto o3d_cloud = std::make_shared<open3d::geometry::PointCloud>();
    o3d_cloud->points_.reserve(pcl_cloud.size());
    
    for (const auto& point : pcl_cloud.points) {
        o3d_cloud->points_.push_back(Eigen::Vector3d(point.x, point.y, point.z));
    }
    
    return o3d_cloud;
}

// 辅助函数：Open3D变换矩阵转Eigen
Eigen::Matrix4f open3dToEigen(const Eigen::Matrix4d& o3d_trans) {
    return o3d_trans.cast<float>();
}

BackEnd::BackEnd(const std::string &config_file_path, const std::string &prefix)
    : ModuleBase(config_file_path, prefix, "Back End Module") {
    std::vector<double> extrin_v;
    readParam("extrin_r", extrin_v, std::vector<double>());
    extrin_r.setIdentity();
    extrin_t.setZero();
    if (extrin_v.size() == 9) {
        Eigen::Matrix3d extrin_r33;
        extrin_r33 << extrin_v[0], extrin_v[1], extrin_v[2], extrin_v[3], extrin_v[4], extrin_v[5], extrin_v[6],
            extrin_v[7], extrin_v[8];
        extrin_r = extrin_r33;
    } else if (extrin_v.size() == 3) {
        extrin_r.x() = extrin_v[0];
        extrin_r.y() = extrin_v[1];
        extrin_r.z() = extrin_v[2];
        extrin_r.w() = extrin_v[3];
    }
    readParam("extrin_t", extrin_v, std::vector<double>());
    if (extrin_v.size() == 3) {
        extrin_t << extrin_v[0], extrin_v[1], extrin_v[2];
    }
    extrin_t = {0,0,0.28};
    std::cout << extrin_r.matrix()<<std::endl;
    pgo = std::make_shared<PoseGraphOpt>();
    voxel_filter.setLeafSize(0.1, 0.1, 0.1);
}

BackEnd::~BackEnd() { saveResult(); }

bool BackEnd::addFrame(PCLPointCloud &opt_map, PCLPointCloud &cloud, Pose &pose) {
    // 检查是否符合添加关键帧的条件：
    static int cnt = 0;
    if (cnt > 100 || poses.empty() || (poses.back().position - pose.position).norm() > 1.0) {  // check for keyframe
        sc_manager.makeAndSaveScancontextAndKeys(cloud);
        clouds.push_back(cloud);
        poses.push_back(pose);
        cnt = 0;
        auto res = sc_manager.detectLoopClosureID();
        // 检测到回环

        if (res.first != -1) {
            // 进行匹配
            Eigen::Matrix4f trans_icp;
            Eigen::Matrix4d T_L_I, T_I_L, T_c;
            if (scanRegister(trans_icp, clouds.size() - 1, res.first, res.second)) {
                // 匹配成功，计算约束
                BinaryEdge be;
                be.from_vertex = clouds.size() - 1;
                be.to_vertex = res.first;
                T_L_I.setIdentity();
                T_L_I.block<3, 3>(0, 0) = extrin_r.toRotationMatrix();
                T_L_I.block<3, 1>(0, 3) = extrin_t;
                // 使用性质进行取逆
                T_I_L.block<3, 3>(0, 0) = extrin_r.conjugate().toRotationMatrix();
                T_I_L.block<3, 1>(0, 3) = extrin_r.conjugate() * extrin_t * (-1.0);
                // 计算T_c
                T_c = T_L_I * trans_icp.cast<double>() * T_I_L;
                be.constraint.rotation = Eigen::Quaterniond(T_c.block<3, 3>(0, 0));
                be.constraint.position = T_c.block<3, 1>(0, 3);
                binary_edges.push_back(be);
                // copy 所有的位姿
                std::vector<Pose> copy_poses = poses;
                if (pgo->slove(copy_poses, binary_edges)) {
                    opt_map.clear();
                    for (int i = 0; i < clouds.size(); i++) {
                        Eigen::Matrix4f T_I_W, T_L_W;
                        T_I_W.setIdentity();
                        T_I_W.block<3, 3>(0, 0) = copy_poses[i].rotation.cast<float>().toRotationMatrix();
                        T_I_W.block<3, 1>(0, 3) = copy_poses[i].position.cast<float>();
                        T_L_W = T_I_W * T_L_I.cast<float>();
                        PCLPointCloud global_cloud;
                        voxel_filter.setInputCloud(clouds[i].makeShared());
                        voxel_filter.filter(global_cloud);
                        pcl::transformPointCloud(global_cloud, global_cloud, T_L_W);
                        opt_map += global_cloud;
                    }
                    voxel_filter.setInputCloud(opt_map.makeShared());
                    voxel_filter.filter(opt_map);
                    // update pose
                    for (int i = 0; i < poses.size(); i++) {
                        poses[i] = copy_poses[i];
                    }

                    return true;
                } else {
                    binary_edges.pop_back();
                }
            }
        }
    }
    cnt++;

    return false;
}

bool BackEnd::scanRegister(Eigen::Matrix4f &match_result, int from_id, int to_id, float angle) {
    // 1. 构造局部地图 (local map)
    PCLPointCloud local_map;
    int start_id = std::max(0, to_id - 10);
    int end_id = std::min((int)clouds.size() - 1, to_id + 10);

    for (int i = start_id; i <= end_id; i += 4) {
        if (i < 0 || i >= (int)clouds.size())
            continue;

        // 取该关键帧的位姿
        Eigen::Matrix4f T_I_W = Eigen::Matrix4f::Identity();
        T_I_W.block<3, 3>(0, 0) = poses[i].rotation.cast<float>().toRotationMatrix();
        T_I_W.block<3, 1>(0, 3) = poses[i].position.cast<float>();

        // 雷达到IMU的外参 (T_L_I)
        Eigen::Matrix4f T_L_I = Eigen::Matrix4f::Identity();
        T_L_I.block<3, 3>(0, 0) = extrin_r.toRotationMatrix().cast<float>();
        T_L_I.block<3, 1>(0, 3) = extrin_t.cast<float>();

        // 点云从L系到世界系
        Eigen::Matrix4f T_L_W = T_I_W * T_L_I;

        PCLPointCloud transformed;
        pcl::transformPointCloud(clouds[i], transformed, T_L_W);
        local_map += transformed;
    }
    
    PCLPointCloud local_map_filtered;
    voxel_filter.setInputCloud(local_map.makeShared());
    voxel_filter.filter(local_map_filtered);

    // 如果local_map为空，就退回到scan-to-scan
    if (local_map_filtered.empty()) {
        std::cerr << "[scanRegister] local_map empty, fallback scan-to-scan" << std::endl;
        
        // 转换为Open3D点云
        auto source_o3d = pclToOpen3d(clouds[from_id]);
        auto target_o3d = pclToOpen3d(clouds[to_id]);
        
        // 体素下采样
        source_o3d = source_o3d->VoxelDownSample(0.5);
        target_o3d = target_o3d->VoxelDownSample(0.5);
        
        // 估计法向量
        source_o3d->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(1.0, 30));
        target_o3d->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(1.0, 30));
        
        // ICP配准
        auto result = open3d::pipelines::registration::RegistrationICP(
            *source_o3d, *target_o3d, 1.0,
            Eigen::Matrix4d::Identity(),
            open3d::pipelines::registration::TransformationEstimationPointToPlane(),
            open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 30));
        
        match_result = result.transformation_.cast<float>();
        std::cout << "Open3D ICP fitness: " << result.fitness_ 
                  << " inlier_rmse: " << result.inlier_rmse_ << std::endl;
        
        return result.fitness_ > 0.3 && result.inlier_rmse_ < 0.5;
    }

    // 2. 转换为Open3D点云
    auto source_o3d = pclToOpen3d(clouds[from_id]);
    auto target_o3d = pclToOpen3d(local_map_filtered);
    
    // 3. 体素下采样
    source_o3d = source_o3d->VoxelDownSample(0.3);
    target_o3d = target_o3d->VoxelDownSample(0.3);
    
    // 4. 估计法向量（用于Point-to-Plane ICP）
    source_o3d->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(1.0, 30));
    target_o3d->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(1.0, 30));
    
    // 5. 计算初始位姿 (ScanContext提供的角度)
    Eigen::AngleAxisd init_rotation(-1 * angle, Eigen::Vector3d::UnitZ());
    Eigen::Matrix4d initial_transform = Eigen::Matrix4d::Identity();
    initial_transform.block<3, 3>(0, 0) = init_rotation.toRotationMatrix();
    
    // 6. Point-to-Plane ICP配准
    auto result = open3d::pipelines::registration::RegistrationICP(
        *source_o3d, *target_o3d, 
        5.0,  // max_correspondence_distance
        initial_transform,
        open3d::pipelines::registration::TransformationEstimationPointToPlane(),
        open3d::pipelines::registration::ICPConvergenceCriteria(1e-6, 1e-6, 50));
    
    // 7. 获取配准结果
    match_result = result.transformation_.cast<float>();
    
    std::cout << "Open3D ICP fitness: " << result.fitness_ 
              << " inlier_rmse: " << result.inlier_rmse_ << std::endl;
    
    // 8. 判断配准是否成功
    // fitness: 内点比例，inlier_rmse: 内点的均方根误差
    return result.fitness_ > 0.3 && result.inlier_rmse_ < 0.5;
}

void BackEnd::saveResult() {
    std::string save_dir = "/result/";
    system(("mkdir -p " + save_dir + "scan").c_str());

    // === 保存点云 ===
    for (size_t i = 0; i < clouds.size(); ++i) {
        std::string filename = save_dir + "scan/" + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(filename, clouds[i]);
    }

    // === 保存 g2o ===
    std::ofstream fout(save_dir + "pose_graph.g2o");
    if (!fout.is_open()) {
        std::cerr << "Cannot open g2o file for writing!" << std::endl;
        return;
    }

    fout << std::fixed << std::setprecision(6);

    // 顶点
    for (size_t i = 0; i < poses.size(); ++i) {
        const auto &p = poses[i];
        fout << "VERTEX_SE3:QUAT " << i << " " << p.position.x() << " " << p.position.y() << " " << p.position.z()
             << " " << p.rotation.x() << " " << p.rotation.y() << " " << p.rotation.z() << " " << p.rotation.w()
             << "\n";
    }

    // 原始二元边
    for (size_t i = 0; i < binary_edges.size(); ++i) {
        const auto &e = binary_edges[i];
        fout << "EDGE_SE3:QUAT " << e.from_vertex << " " << e.to_vertex << " " << e.constraint.position.x() << " "
             << e.constraint.position.y() << " " << e.constraint.position.z() << " " << e.constraint.rotation.x() << " "
             << e.constraint.rotation.y() << " " << e.constraint.rotation.z() << " " << e.constraint.rotation.w();

        // 信息矩阵（只输出上三角部分）
        for (int r = 0; r < 6; r++) {
            for (int c = r; c < 6; c++) {
                fout << " " << e.information(r, c);
            }
        }
        fout << "\n";
    }

    // === 顺序边（连续顶点） ===
    for (size_t i = 0; i + 1 < poses.size(); ++i) {
        const auto &p_from = poses[i];
        const auto &p_to = poses[i + 1];

        // 计算相对位姿 T_from_to = inv(T_from) * T_to
        Eigen::Quaterniond q_rel = p_from.rotation.conjugate() * p_to.rotation;
        Eigen::Vector3d t_rel = p_from.rotation.conjugate() * (p_to.position - p_from.position);

        fout << "EDGE_SE3:QUAT " << i << " " << (i + 1) << " " << t_rel.x() << " " << t_rel.y() << " " << t_rel.z()
             << " " << q_rel.x() << " " << q_rel.y() << " " << q_rel.z() << " " << q_rel.w();

        // 顺序边信息矩阵可以设置单位矩阵
        for (int r = 0; r < 6; r++)
            for (int c = r; c < 6; c++) fout << " " << (r == c ? 1.0 : 0.0);

        fout << "\n";
    }

    fout.close();
    std::cout << "Result saved to " << save_dir << std::endl;
}

}  // namespace IESKFSlam