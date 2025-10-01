#pragma once
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>

#include "ieskf_slam/type/base_type.h"
#include "pcl_conversions/pcl_conversions.h"
namespace ROSNoetic {
using LidarMsgVariant = std::variant<sensor_msgs::PointCloud2, livox_ros_driver::CustomMsg>;
class CommonLidarProcessInterface {
   public:
    virtual bool process(const LidarMsgVariant &msg, IESKFSlam::PointCloud &cloud, const double &time_unit) = 0;
    int num_scans_;
    int point_filter_num_;
    double blind_;

   private:
};
}  // namespace ROSNoetic