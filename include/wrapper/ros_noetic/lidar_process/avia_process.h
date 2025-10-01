#pragma once
#include <execution>

#include "common_lidar_process_interface.h"
#include "ieskf_slam/type/point.h"
#include "ieskf_slam/type/pointcloud.h"

namespace ROSNoetic {

class AVIAProcess : public CommonLidarProcessInterface {
   public:
    AVIAProcess(int num_scans = 6, int point_filter_num = 1, double blind = 1.0)
        : num_scans_(num_scans), point_filter_num_(point_filter_num), blind_(blind) {
        // cloud_full_ 和 cloud_out_ 会动态resize
    }

    bool process(const LidarMsgVariant& msg, IESKFSlam::PointCloud& cloud, const double& time_unit) override {
        return std::visit(
            [&](auto&& m) -> bool {
                using T = std::decay_t<decltype(m)>;
                if constexpr (std::is_same_v<T, livox_ros_driver::CustomMsg>) {
                    // 清空输出点云容器
                    cloud_out_.clear();
                    cloud_full_.clear();

                    int plsize = m.point_num;
                    cloud_out_.reserve(plsize);
                    cloud_full_.resize(plsize);

                    std::vector<bool> is_valid_pt(plsize, false);
                    std::vector<uint> index(plsize - 1);
                    for (uint i = 0; i < plsize - 1; ++i) {
                        index[i] = i + 1;  // 从1开始
                    }

                    std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const uint& i) {
                        if ((m.points[i].line < num_scans_) &&
                            ((m.points[i].tag & 0x30) == 0x10 || (m.points[i].tag & 0x30) == 0x00)) {
                            if (i % point_filter_num_ == 0) {
                                cloud_full_[i].x = m.points[i].x;
                                cloud_full_[i].y = m.points[i].y;
                                cloud_full_[i].z = m.points[i].z;
                                cloud_full_[i].intensity = m.points[i].reflectivity;
                                cloud_full_[i].offset_time = m.points[i].offset_time;  // ms
                                // 注意这里的优先级问题，严格保持你的原逻辑写法
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

                    // 这里你可以选择把 cloud_out_ 赋值到外部 cloud 结构中，
                    // 下面是一个示例，需要根据你 IESKFSlam::PointCloud 的具体结构调整
                    cloud.cloud_ptr->clear();
                    for (auto& pt : cloud_out_.points) {
                        IESKFSlam::Point p;
                        p.x = pt.x;
                        p.y = pt.y;
                        p.z = pt.z;
                        p.intensity = pt.intensity;
                        p.offset_time = pt.offset_time;
                        // curvature字段是否传递看需求
                        cloud.cloud_ptr->push_back(p);
                    }
                    cloud.time_stamp.fromSec(m.header.stamp.toSec());

                    return true;
                } else {
                    return false;  // 非CustomMsg直接返回false
                }
            },
            msg);
    }

   private:
    int num_scans_;
    int point_filter_num_;
    double blind_;

    IESKFSlam::PCLPointCloud cloud_out_;
    IESKFSlam::PCLPointCloud cloud_full_;
};
}  // namespace ROSNoetic