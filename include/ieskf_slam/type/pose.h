/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-09 08:42:00
 * @LastEditors: MengKai
 * @LastEditTime: 2023-06-09 08:43:15
 */
#pragma once
#include <Eigen/Dense>

#include "timestamp.h"
namespace IESKFSlam {
struct Pose {
    TimeStamp time_stamp;
    Eigen::Quaterniond rotation;
    Eigen::Vector3d position;

};
// 二元边结构体（回环约束）
struct BinaryEdge {
    int from_vertex;  // 边的起始顶点索引
    int to_vertex;    // 边的目标顶点索引
    Pose constraint;  // 两个顶点之间的相对位姿约束 T_from_to

    // 可选：约束的信息矩阵（协方差的逆）
    Eigen::Matrix<double, 6, 6> information;

    BinaryEdge(int from = -1, int to = -1, const Pose& pose = Pose())
        : from_vertex(from), to_vertex(to), constraint(pose) {
        information.setIdentity();
    }

    // BinaryEdge(int from, int to, const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
    //     : from_vertex(from), to_vertex(to), constraint(pos, rot) {
    //     information.setIdentity();
    // }
};
}  // namespace IESKFSlam