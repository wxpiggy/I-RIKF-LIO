#pragma once

#include <mutex>
#include <string>
#include <sstream>
#include <fstream>
#include <stdio.h>
#include <unordered_map>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <pcl/common/io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "omp.h"

#define HASH_P 116101
#define MAX_N 10000000000
#define MAX_FRAME_N 20000

struct ConfigSetting {
  /* for point cloud pre-preocess*/
  int stop_skip_enable_ = 0;
  double ds_size_ = 0.5;
  int maximum_corner_num_ = 30;

  /* for key points*/
  double plane_merge_normal_thre_;
  double plane_merge_dis_thre_;
  double plane_detection_thre_ = 0.01;
  double voxel_size_ = 1.0;
  int voxel_init_num_ = 10;
  double proj_image_resolution_ = 0.5;
  double proj_dis_min_ = 0.2;
  double proj_dis_max_ = 5;
  double corner_thre_ = 10;

  /* for STD */
  int descriptor_near_num_ = 10;
  double descriptor_min_len_ = 1;
  double descriptor_max_len_ = 10;
  double non_max_suppression_radius_ = 3.0;
  double std_side_resolution_ = 0.2;
  bool intensity_enhanced_ = false;

  /* for place recognition*/
  int skip_near_num_ = 50;
  int candidate_num_ = 50;
  int sub_frame_num_ = 10;
  double rough_dis_threshold_ = 0.03;
  double vertex_diff_threshold_ = 0.7;
  double icp_threshold_ = 0.5;
  double normal_threshold_ = 0.1;
  double dis_threshold_ = 0.3;
};

struct STDesc {
  Eigen::Vector3d side_length_;
  Eigen::Vector3d angle_;
  Eigen::Vector3d center_;
  unsigned int frame_id_;

  Eigen::Vector3d vertex_A_;
  Eigen::Vector3d vertex_B_;
  Eigen::Vector3d vertex_C_;
  Eigen::Vector3d vertex_attached_;
};

struct Plane {
  pcl::PointXYZINormal p_center_;
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Matrix3d covariance_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float intercept_ = 0;
  int id_ = 0;
  int sub_plane_num_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
};

struct STDMatchList {
  std::vector<std::pair<STDesc, STDesc>> match_list_;
  std::pair<int, int> match_id_;
  double mean_dis_;
};

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

struct M_POINT {
  float xyz[3];
  float intensity;
  int count = 0;
};

template <>
struct std::hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC& s) const {
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

class STDesc_LOC {
 public:
  int64_t x, y, z, a, b, c;

  STDesc_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0,
             int64_t va = 0, int64_t vb = 0, int64_t vc = 0)
      : x(vx), y(vy), z(vz), a(va), b(vb), c(vc) {}

  bool operator==(const STDesc_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

template <>
struct std::hash<STDesc_LOC> {
  int64_t operator()(const STDesc_LOC& s) const {
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

class OctoTree {
 public:
  ConfigSetting config_setting_;
  std::vector<Eigen::Vector4d> voxel_points_;
  Plane* plane_ptr_;
  int layer_;
  int octo_state_;
  int merge_num_ = 0;
  bool is_project_ = false;
  std::vector<Eigen::Vector3d> proj_normal_vec_;

  bool is_check_connect_[6];
  bool connect_[6];
  OctoTree* connect_tree_[6];

  bool is_publish_ = false;
  OctoTree* leaves_[8];
  double voxel_center_[3];
  float quater_length_;
  bool init_octo_;
  OctoTree(const ConfigSetting& config_setting)
      : config_setting_(config_setting) {
    voxel_points_.clear();
    octo_state_ = 0;
    layer_ = 0;
    init_octo_ = false;
    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    for (int i = 0; i < 6; i++) {
      is_check_connect_[i] = false;
      connect_[i] = false;
      connect_tree_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }
  void init_plane();
  void init_octo_tree();
};

// Function declarations
void down_sampling_voxel(pcl::PointCloud<pcl::PointXYZI>& pl_feat, double voxel_size);
void read_parameters(ros::NodeHandle& nh, ConfigSetting& config_setting);
pcl::PointXYZI vec2point(const Eigen::Vector3d& vec);
Eigen::Vector3d point2vec(const pcl::PointXYZI& pi);
void publish_std_pairs(const std::vector<std::pair<STDesc, STDesc>>& match_std_pairs,
                       const ros::Publisher& std_publisher);
bool attach_greater_sort(std::pair<double, int> a, std::pair<double, int> b);

struct PlaneSolver {
  PlaneSolver(Eigen::Vector3d curr_point_, Eigen::Vector3d curr_normal_,
              Eigen::Vector3d target_point_, Eigen::Vector3d target_normal_)
      : curr_point(curr_point_), curr_normal(curr_normal_), target_point(target_point_), target_normal(target_normal_) {};
  template <typename T>
  bool operator()(const T* q, const T* t, T* residual) const {
    Eigen::Quaternion<T> q_w_curr{q[0], q[1], q[2], q[3]};
    Eigen::Matrix<T, 3, 1> t_w_curr{t[0], t[1], t[2]};
    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()),
                              T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> point_w;
    point_w = q_w_curr * cp + t_w_curr;
    Eigen::Matrix<T, 3, 1> point_target(
        T(target_point.x()), T(target_point.y()), T(target_point.z()));
    Eigen::Matrix<T, 3, 1> norm(T(target_normal.x()), T(target_normal.y()),
                                T(target_normal.z()));
    residual[0] = norm.dot(point_w - point_target);
    return true;
  }

  static ceres::CostFunction* Create(const Eigen::Vector3d curr_point_,
                                     const Eigen::Vector3d curr_normal_,
                                     Eigen::Vector3d target_point_,
                                     Eigen::Vector3d target_normal_) {
    return (
        new ceres::AutoDiffCostFunction<PlaneSolver, 1, 4, 3>(new PlaneSolver(
            curr_point_, curr_normal_, target_point_, target_normal_)));
  }

  Eigen::Vector3d curr_point;
  Eigen::Vector3d curr_normal;
  Eigen::Vector3d target_point;
  Eigen::Vector3d target_normal;
};

class STDescManager {
 public:
  STDescManager() = default;
  STDescManager(ConfigSetting& config_setting) : config_setting_(config_setting) {};

  void GenerateSTDescs(pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                       std::vector<STDesc>& stds_vec);
  void SearchLoop(const std::vector<STDesc>& stds_vec,
                  std::pair<int, double>& loop_result,
                  std::pair<Eigen::Vector3d, Eigen::Matrix3d>& loop_transform,
                  std::vector<std::pair<STDesc, STDesc>>& loop_std_pair);
  void AddSTDescs(const std::vector<STDesc>& stds_vec);
  void PlaneGeomrtricIcp(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_cloud,
                         const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_cloud,
                         std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform);

  ConfigSetting config_setting_;
  unsigned int current_frame_id_ = 0u;
  std::unordered_map<STDesc_LOC, std::vector<STDesc>> data_base_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> key_cloud_vec_;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> corner_cloud_vec_;
  std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> plane_cloud_vec_;

 private:
  void init_voxel_map(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                      std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map);
  void build_connection(std::unordered_map<VOXEL_LOC, OctoTree*>& feat_map);
  void getPlane(const std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
                pcl::PointCloud<pcl::PointXYZINormal>::Ptr& plane_cloud);
  void corner_extractor(std::unordered_map<VOXEL_LOC, OctoTree*>& voxel_map,
                        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                        pcl::PointCloud<pcl::PointXYZINormal>::Ptr& corner_points);
  void extract_corner(const Eigen::Vector3d& proj_center,
                      const Eigen::Vector3d proj_normal,
                      const std::vector<Eigen::Vector4d> proj_points,
                      pcl::PointCloud<pcl::PointXYZINormal>::Ptr& corner_points);
  void non_maxi_suppression(pcl::PointCloud<pcl::PointXYZINormal>::Ptr& corner_points);
  void build_stdesc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& corner_points,
                    std::vector<STDesc>& stds_vec);
  void candidate_selector(const std::vector<STDesc>& stds_vec,
                          std::vector<STDMatchList>& candidate_matcher_vec);
  void candidate_verify(const STDMatchList& candidate_matcher,
                        double& verify_score,
                        std::pair<Eigen::Vector3d, Eigen::Matrix3d>& relative_pose,
                        std::vector<std::pair<STDesc, STDesc>>& sucess_match_vec);
  void triangle_solver(std::pair<STDesc, STDesc>& std_pair,
                       Eigen::Vector3d& t,
                       Eigen::Matrix3d& rot);
  double plane_geometric_verify(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source_cloud,
                                const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target_cloud,
                                const std::pair<Eigen::Vector3d, Eigen::Matrix3d>& transform);
};