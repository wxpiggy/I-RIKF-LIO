#pragma once
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/pointcloud.h"
#include "ieskf_slam/type/base_type.h"
#include "pcl/common/transforms.h"
#include "ieskf_slam/math/math.h"

namespace IESKFSlam {
    class RectMapManager : private ModuleBase {
    private:
        PCLPointCloudPtr local_map_ptr;
        KDTreePtr kdtree_ptr;
        float map_side_length_2;
        float map_resolution;
    public:
        RectMapManager(const std::string &config_path,
                       const std::string &prefix);
        ~RectMapManager();
        void reset();
        void addScan(PCLPointCloudPtr curr_scan,
                     const Eigen::Quaterniond &att_q,
                     const Eigen::Vector3d &pos_t);
        PCLPointCloudConstPtr getLocalMap();
        KDTreeConstPtr readKDtree();
    };
}  // namespace IESKFSlam