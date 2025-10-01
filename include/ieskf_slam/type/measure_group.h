#pragma once
#include <deque>

#include "ieskf_slam/type/imu.h"
#include "ieskf_slam/type/pointcloud.h"
namespace IESKFSlam {
struct MeasureGroup {
    double lidar_begin_time;
    double lidar_end_time;
    std::deque<IMU> imus;
    PointCloud cloud;
};

}  // namespace IESKFSlam