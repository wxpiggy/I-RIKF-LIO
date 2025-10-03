#pragma once
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <execution>

#include "ieskf_slam/type/point.h"
#include "ieskf_slam/type/pointcloud.h"

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

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
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)
// clang-format on

namespace ROSNoetic {

class LidarPreprocessor {
   public:
    LidarPreprocessor(int num_scans = 6, int point_filter_num = 1, double blind = 1.0)
        : num_scans_(num_scans), point_filter_num_(point_filter_num), blind_(blind) {}

    // AVIA雷达处理 - offset_time已经是纳秒
    bool aviaHandler(const livox_ros_driver::CustomMsg& msg, IESKFSlam::PointCloud& cloud) {
        cloud_out_.clear();
        cloud_full_.clear();

        int plsize = msg.point_num;
        cloud_out_.reserve(plsize);
        cloud_full_.resize(plsize);

        std::vector<bool> is_valid_pt(plsize, false);
        std::vector<uint> index(plsize - 1);
        for (uint i = 0; i < plsize - 1; ++i) {
            index[i] = i + 1;
        }

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint& i) {
            if ((msg.points[i].line < num_scans_) &&
                ((msg.points[i].tag & 0x30) == 0x10 || (msg.points[i].tag & 0x30) == 0x00)) {
                if (i % point_filter_num_ == 0) {
                    cloud_full_[i].x = msg.points[i].x;
                    cloud_full_[i].y = msg.points[i].y;
                    cloud_full_[i].z = msg.points[i].z;
                    cloud_full_[i].intensity = msg.points[i].reflectivity;
                    cloud_full_[i].offset_time = msg.points[i].offset_time;  // 已经是ns

                    if ((abs(cloud_full_[i].x - cloud_full_[i - 1].x) > 1e-7) ||
                        (abs(cloud_full_[i].y - cloud_full_[i - 1].y) > 1e-7) ||
                        (abs(cloud_full_[i].z - cloud_full_[i - 1].z) > 1e-7) &&
                            (cloud_full_[i].x * cloud_full_[i].x + cloud_full_[i].y * cloud_full_[i].y +
                                 cloud_full_[i].z * cloud_full_[i].z >
                             (blind_ * blind_))) {
                        is_valid_pt[i] = true;
                    }
                }
            }
        });

        for (uint i = 1; i < plsize; ++i) {
            if (is_valid_pt[i]) {
                cloud_out_.points.push_back(cloud_full_[i]);
            }
        }

        cloud.cloud_ptr->clear();
        for (auto& pt : cloud_out_.points) {
            IESKFSlam::Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            p.offset_time = pt.offset_time;
            cloud.cloud_ptr->push_back(p);
        }
        cloud.time_stamp.fromSec(msg.header.stamp.toSec());

        return true;
    }

    // Velodyne雷达处理 -
    bool velodyneHandler(const sensor_msgs::PointCloud2& msg, IESKFSlam::PointCloud& cloud, double time_scale) {
        pcl::PointCloud<velodyne_ros::Point> pcl_cloud;
        pcl::fromROSMsg(msg, pcl_cloud);
        cloud.cloud_ptr->clear();

        if (pcl_cloud.empty())
            return false;

        double end_time = msg.header.stamp.toSec();
        double start_time = end_time + pcl_cloud[0].time * time_scale;

        for (size_t i = 0; i < pcl_cloud.size(); ++i) {
            if (i % point_filter_num_ != 0)
                continue;

            const auto& p = pcl_cloud[i];
            double dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            if (dist_sq < blind_ * blind_)
                continue;

            IESKFSlam::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.intensity = p.intensity;
            point.ring = p.ring;
            point.offset_time = (p.time * 1e-6 + end_time - start_time) * 1e9;  // μs -> ns
            cloud.cloud_ptr->push_back(point);
        }

        cloud.time_stamp.fromSec(start_time);
        return true;
    }

    // Ouster雷达处理 - t已经是纳秒
    // bool ousterHandler(const sensor_msgs::PointCloud2& msg, IESKFSlam::PointCloud& cloud) {
    //     pcl::PointCloud<ouster_ros::Point> pcl_cloud;
    //     pcl::fromROSMsg(msg, pcl_cloud);
    //     cloud.cloud_ptr->clear();

    //     if (pcl_cloud.empty())
    //         return false;

    //     // 起始点时间（ns）
    //     uint64_t t0 = pcl_cloud[0].t;
    //     double blind_sq = blind_ * blind_;

    //     for (size_t i = 0; i < pcl_cloud.size(); ++i) {
    //         if (i % point_filter_num_ != 0)
    //             continue;

    //         const auto& p = pcl_cloud[i];
    //         double dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
    //         if (dist_sq < blind_sq)
    //             continue;

    //         IESKFSlam::Point point;
    //         point.x = p.x;
    //         point.y = p.y;
    //         point.z = p.z;
    //         point.intensity = p.intensity;
    //         point.ring = p.ring;

    //         // 相对起始点的时间，单位 ns
    //         point.offset_time = (i % 2048)/20480.0 * 1e-6 ;

    //         cloud.cloud_ptr->push_back(point);
    //     }

    //     // 设置帧时间戳（ROS 需要秒单位）
    //     cloud.time_stamp.fromSec(msg.header.stamp.toSec() + static_cast<double>(t0) * 1e-9);

    //     return true;
    // }
    bool ousterHandler(const sensor_msgs::PointCloud2& msg, IESKFSlam::PointCloud& cloud) {
        pcl::PointCloud<ouster_ros::Point> pcl_cloud;
        pcl::fromROSMsg(msg, pcl_cloud);
        cloud.cloud_ptr->clear();

        if (pcl_cloud.empty())
            return false;

        uint64_t t0 = pcl_cloud[0].t;
        double blind_sq = blind_ * blind_;
        // std::sort(pcl_cloud.begin(), pcl_cloud.end(),
        //           [](const ouster_ros::Point& a, const ouster_ros::Point& b) { return (a.t < b.t); });
        // std::cout << pcl_cloud.size()<<std::endl;
        // std::cout << pcl_cloud[1023].t << std::endl;
        // std::cout << pcl_cloud[2047].t << std::endl;
        // std::cout << pcl_cloud[4095].t << std::endl;
        for (size_t i = 0; i < pcl_cloud.size(); ++i) {
            if (i % point_filter_num_ != 0)
                continue;

            const auto& p = pcl_cloud[i];
            double dist_sq = p.x * p.x + p.y * p.y + p.z * p.z;
            if (dist_sq < blind_sq)
                continue;

            IESKFSlam::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.intensity = p.intensity;
            point.ring = p.ring;
            point.offset_time = (i % 2048) / 20480.0 * 1e6;
            cloud.cloud_ptr->push_back(point);
        }
        cloud.time_stamp.fromSec(msg.header.stamp.toSec() + t0 * 1e-9);
        return true;
    }

   private:
    int num_scans_;
    int point_filter_num_;
    double blind_;

    IESKFSlam::PCLPointCloud cloud_out_;
    IESKFSlam::PCLPointCloud cloud_full_;
};

}  // namespace ROSNoetic