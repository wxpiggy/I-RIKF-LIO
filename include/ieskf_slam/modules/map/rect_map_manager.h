#pragma once
#include "ieskf_slam/math/math.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/base_type.h"
#include "ieskf_slam/type/pointcloud.h"
#include "pcl/common/transforms.h"

namespace IESKFSlam {
class MapManager : private ModuleBase {
   private:
    PCLPointCloudPtr local_map_ptr;
    PCLPointCloudPtr global_map_ptr;
    KDTreePtr kdtree_ptr;


   public:
       float map_side_length_2;
    float map_resolution;
    MapManager(const std::string &config_path, const std::string &prefix);
    ~MapManager();
    void reset();
    void addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q, const Eigen::Vector3d &pos_t);
    PCLPointCloudConstPtr getLocalMap();
    const KDTreePtr readKDtree();
    // KDTreeConstPtr readKDtree();
};
}  // namespace IESKFSlam