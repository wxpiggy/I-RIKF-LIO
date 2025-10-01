#pragma once

#define PCL_NO_PRECOMPILE


#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>

#include <pcl/impl/pcl_base.hpp>

#include "pcl/point_types.h"

namespace IESKFSlam {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;

    float intensity = 0.0f;
    std::uint32_t offset_time = 0;
    std::int32_t ring = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Point() = default;  // 显式声明默认构造函数，初始化列表用成员默认值完成
};
}  // namespace IESKFSlam

POINT_CLOUD_REGISTER_POINT_STRUCT(IESKFSlam::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint32_t, offset_time,
                                                                                     offset_time)(std::int32_t, ring,
                                                                                                  ring))