#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "common_lidar_process_interface.h"

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(float, time, time)(std::uint16_t,
                                                                                                        ring, ring))

namespace ROSNoetic {

class VelodyneProcess : public CommonLidarProcessInterface {
   public:
    VelodyneProcess(int num_scans = 32, int point_filter_num = 1, double blind = 4) {
        num_scans_ = num_scans;
        point_filter_num_ = point_filter_num;
        blind_ = blind;
    }

    bool process(const LidarMsgVariant &msg, IESKFSlam::PointCloud &cloud, const double &time_unit) override {
        return std::visit(
            [&](auto &&m) -> bool {
                using T = std::decay_t<decltype(m)>;

                // 只处理 PointCloud2
                if constexpr (std::is_same_v<T, sensor_msgs::PointCloud2>) {
                    pcl::PointCloud<velodyne_ros::Point> rs_cloud;
                    pcl::fromROSMsg(m, rs_cloud);
                    cloud.cloud_ptr->clear();

                    if (rs_cloud.empty())
                        return false;

                    double end_time = m.header.stamp.toSec();
                    double start_time = end_time + rs_cloud[0].time * time_unit;

                    for (size_t i = 0; i < rs_cloud.size(); ++i) {
                        if (i % point_filter_num_ != 0)
                            continue;

                        const auto &p = rs_cloud[i];
                        double dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
                        if (dist_sq < blind_ * blind_)
                            continue;

                        IESKFSlam::Point point;
                        double point_time = p.time * time_unit + end_time;

                        point.x = p.x;
                        point.y = p.y;
                        point.z = p.z;
                        point.intensity = p.intensity;
                        point.ring = p.ring;
                        point.offset_time = (point_time - start_time) * 1e9;
                        cloud.cloud_ptr->push_back(point);
                    }

                    cloud.time_stamp.fromSec(start_time);
                    return true;
                }
                // 其他类型不处理
                else {
                    return false;
                }
            },
            msg);
    }
};

}  // namespace ROSNoetic