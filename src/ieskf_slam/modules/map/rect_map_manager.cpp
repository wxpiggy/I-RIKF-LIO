#include "ieskf_slam/modules/map/rect_map_manager.h"

#include <memory>
#include <vector>

#include "ieskf_slam/math/math.h"
#include "ieskf_slam/type/point.h"
#include "pcl/common/transforms.h"
namespace IESKFSlam {
MapManager::MapManager(const std::string &config_file_path, const std::string &prefix)
    : ModuleBase(config_file_path, prefix, "MapManager") {

    local_map_ptr = pcl::make_shared<PCLPointCloud>();
    global_map_ptr = pcl::make_shared<PCLPointCloud>();
    kdtree_ptr = std::make_shared<KD_TREE<IESKFSlam::Point>>();
    kdtree_ptr->InitializeKDTree();
    readParam("map_side_length_2", map_side_length_2, 1000.0f);
    readParam("map_resolution", map_resolution, 0.5f);
    // map_side_length_2 = 1000;
    // map_resolution = 0.5;
}

MapManager::~MapManager() {}
void MapManager::addScan(PCLPointCloudPtr curr_scan, const Eigen::Quaterniond &att_q, const Eigen::Vector3d &pos_t) {
    static int count = 1;
    PCLPointCloud scan;
    pcl::transformPointCloud(*curr_scan, scan, compositeTransform(att_q, pos_t).cast<float>());

    if (local_map_ptr->empty()) {
        *local_map_ptr = scan;
        kdtree_ptr->Build(scan.points);
    } else {
        std::vector<IESKFSlam::Point, Eigen::aligned_allocator<IESKFSlam::Point>> new_points;  // 本次新增点缓存
        for (auto &&point : scan) {
            std::vector<IESKFSlam::Point, Eigen::aligned_allocator<IESKFSlam::Point>> ind;
            std::vector<float> distance;
            kdtree_ptr->Nearest_Search(point, 5, ind, distance);
            if (distance.empty() || distance[0] > map_resolution) {
                new_points.push_back(point);
                local_map_ptr->push_back(point);
            }
        }

        // 删除远处点前先保存被删除点box范围，用于KD树删除
        std::vector<BoxPointType> boxes_to_delete;

        int left = 0, right = local_map_ptr->size() - 1;
        while (left < right) {
            while (left < right && (fabs(local_map_ptr->points[right].x - pos_t.x()) > map_side_length_2 ||
                                    fabs(local_map_ptr->points[right].y - pos_t.y()) > map_side_length_2 ||
                                    fabs(local_map_ptr->points[right].z - pos_t.z()) > map_side_length_2)) {
                // 记录被删除点的包围盒（这里简单用点构建小立方体）
                BoxPointType box;
                float margin = 0.5f;  // 你可以调整边界margin大小
                box.vertex_min[0] = local_map_ptr->points[right].x - margin;
                box.vertex_min[1] = local_map_ptr->points[right].y - margin;
                box.vertex_min[2] = local_map_ptr->points[right].z - margin;
                box.vertex_max[0] = local_map_ptr->points[right].x + margin;
                box.vertex_max[1] = local_map_ptr->points[right].y + margin;
                box.vertex_max[2] = local_map_ptr->points[right].z + margin;
                boxes_to_delete.push_back(box);

                right--;
            }
            while (left < right && (fabs(local_map_ptr->points[left].x - pos_t.x()) < map_side_length_2 &&
                                    fabs(local_map_ptr->points[left].y - pos_t.y()) < map_side_length_2 &&
                                    fabs(local_map_ptr->points[left].z - pos_t.z()) < map_side_length_2))
                left++;

            if (left < right)
                std::swap(local_map_ptr->points[left], local_map_ptr->points[right]);
        }

        local_map_ptr->resize(right + 1);

        // KD树中删除远处点
        if (!boxes_to_delete.empty()) {
            kdtree_ptr->Delete_Point_Boxes(boxes_to_delete);
        }

        // 增量添加新增点，采样开启（true）
        if (!new_points.empty()) {
            kdtree_ptr->Add_Points(new_points, true);
        }
    }
    // if (count % 100 == 0) {
    //     std::vector<IESKFSlam::Point, Eigen::aligned_allocator<IESKFSlam::Point>> points_history;
    //     kdtree_ptr->acquire_removed_points(points_history);
    //     kdtree_ptr->Delete_Points(points_history);
    // }
    // count++;
}

void MapManager::reset() { local_map_ptr->clear(); }
PCLPointCloudConstPtr MapManager::getLocalMap() { return local_map_ptr; }
const KDTreePtr MapManager::readKDtree() { return kdtree_ptr; }

}  // namespace IESKFSlam