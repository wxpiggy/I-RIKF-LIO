#pragma once
#include "ieskf_slam/type/point.h"
#include "ieskf_slam/type/timestamp.h"

namespace IESKFSlam {
using PCLPointCloud = pcl::PointCloud<Point>;
using PCLPointCloudPtr = PCLPointCloud::Ptr;
using PCLPointCloudConstPtr = PCLPointCloud::ConstPtr;
struct PointCloud {
    using Ptr = std::shared_ptr<PointCloud>;  //指针别名
    TimeStamp time_stamp;                     //时间戳
    PCLPointCloudPtr cloud_ptr;               // pcl
    PointCloud() { cloud_ptr = pcl::make_shared<PCLPointCloud>(); }
};

}  // namespace IESKFSlam