#pragma once
#include <cmath>  // isnan, sqrt

#include "ieskf_slam/math/SO3.h"
#include "ieskf_slam/math/geometry.h"
#include "ieskf_slam/modules/ieskf/ieskf.h"
#include "ieskf_slam/type/base_type.h"

namespace IESKFSlam {
class LIOZHModel : public IESKF::CalcZHInterface {
   private:
    const int NEAR_POINTS_NUM = 5;
    using loss_type = triple<Eigen::Vector3d, Eigen::Vector3d, double>;

    // 使用 ikd-Tree 智能指针类型
    KDTreePtr global_map_kdtree_ptr;
    PCLPointCloudPtr current_cloud_ptr;
    PCLPointCloudConstPtr local_map_ptr;

   public:
    using Ptr = std::shared_ptr<LIOZHModel>;

    // prepare 接收 ikd-Tree 智能指针
    void prepare(KDTreePtr kd_tree, PCLPointCloudPtr current_cloud, PCLPointCloudConstPtr local_map) {
        global_map_kdtree_ptr = kd_tree;
        current_cloud_ptr = current_cloud;
        local_map_ptr = local_map;
    }

    bool calculate(const IESKF::State18 &state, Eigen::MatrixXd &Z, Eigen::MatrixXd &H) override {
        std::vector<loss_type> loss_v;
        loss_v.resize(current_cloud_ptr->size());
        std::vector<bool> is_effect_point(current_cloud_ptr->size(), false);
        std::vector<loss_type> loss_real;
        int valid_points_num = 0;

#ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
        for (size_t i = 0; i < current_cloud_ptr->size(); i++) {
            // 1) 变换到世界系
            Point point_imu = current_cloud_ptr->points[i];
            Point point_world;
            point_world = transformPoint(point_imu, state.rotation, state.position);

            // 2) ikd-Tree 最近邻搜索（返回点和平方距离）
            std::vector<Point, Eigen::aligned_allocator<IESKFSlam::Point>> nearest_pts;
            std::vector<float> nearest_sqdist;
            global_map_kdtree_ptr->Nearest_Search(point_world, NEAR_POINTS_NUM, nearest_pts, nearest_sqdist);

            // 3) 距离/数量判断
            if (nearest_sqdist.size() < (size_t)NEAR_POINTS_NUM)
                continue;
            if (std::sqrt(nearest_sqdist[NEAR_POINTS_NUM - 1]) > 5.0)
                continue;

            // 4) 构造邻域点用于平面检验
            std::vector<Point> planar_points;
            planar_points.reserve(NEAR_POINTS_NUM);
            for (int ni = 0; ni < NEAR_POINTS_NUM; ni++) {
                planar_points.push_back(nearest_pts[ni]);
            }

            Eigen::Vector4d pabcd;
            if (planarCheck(planar_points, pabcd, 0.1)) {
                // 5) 点到平面距离
                double pd = point_world.x * pabcd(0) + point_world.y * pabcd(1) + point_world.z * pabcd(2) + pabcd(3);

                loss_type loss;
                loss.thrid = pd;                                       // 残差
                loss.first = {point_imu.x, point_imu.y, point_imu.z};  // imu系点
                loss.second = pabcd.block<3, 1>(0, 0);                 // 平面法向量

                if (std::isnan(pd) || std::isnan(loss.second(0)) || std::isnan(loss.second(1)) ||
                    std::isnan(loss.second(2)))
                    continue;

                double s = 1.0 - 0.9 * fabs(pd) / std::sqrt(loss.first.norm() + 1e-12);
                if (s > 0.9) {
                    valid_points_num++;
                    loss_v[i] = loss;
                    is_effect_point[i] = true;
                }
            }
        }  // end for points

        // 收集有效点
        for (size_t i = 0; i < current_cloud_ptr->size(); i++) {
            if (is_effect_point[i])
                loss_real.push_back(loss_v[i]);
        }
        valid_points_num = static_cast<int>(loss_real.size());

        // 分配 H 和 Z
        H = Eigen::MatrixXd::Zero(valid_points_num, 18);
        Z.resize(valid_points_num, 1);
        for (int vi = 0; vi < valid_points_num; vi++) {
            Eigen::Vector3d dr = -1.0 * loss_real[vi].second.transpose() * state.rotation.toRotationMatrix() *
                                 math::skewSymmetric(loss_real[vi].first);
            H.block<1, 3>(vi, 0) = dr.transpose();
            H.block<1, 3>(vi, 3) = loss_real[vi].second.transpose();
            Z(vi, 0) = loss_real[vi].thrid;
        }

        return true;
    }
};

}  // namespace IESKFSlam