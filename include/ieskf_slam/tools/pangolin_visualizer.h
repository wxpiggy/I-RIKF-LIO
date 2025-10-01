#ifndef PANGOLIN_VISUALIZER_H
#define PANGOLIN_VISUALIZER_H

#include <pangolin/pangolin.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

class PangolinVisualizer {
   public:
    struct Pose {
        Eigen::Vector3d position;
        Eigen::Quaterniond rotation;

        Pose();
        Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot);
    };

    PangolinVisualizer();
    ~PangolinVisualizer();

    void start();
    void stop();

    // 原有接口（保持兼容性）
    void updatePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation);
    void updateCurrentCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>& map);

    // 新增主要接口：更新当前扫描和位姿（推荐使用这个）
    void updateCurrentScan(const pcl::PointCloud<pcl::PointXYZI>& current_scan, const Eigen::Vector3d& position,
                           const Eigen::Quaterniond& rotation);

    // 累积地图控制
    void setAccumulationParams(size_t max_points, int downsample_rate = 1);
    void clearAccumulatedMap();

   private:
    // 渲染相关
    void renderLoop();
    void initializePangolin();

    // 绘制函数
    void drawTrajectory();
    void drawPointCloudColorMode(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void drawCurrentScanHighlighted(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void drawCurrentPose();
    void drawCoordinateFrame();

    // 点云累积
    void accumulatePointCloud(const pcl::PointCloud<pcl::PointXYZI>& scan, const Eigen::Vector3d& position,
                              const Eigen::Quaterniond& rotation);

    // 颜色相关
    void BuildIntensityTable();
    Eigen::Vector4d IntensityToRgbPCL(float norm_val) {
        if (intensity_color_table_pcl_.empty())
            return Eigen::Vector4d(1, 1, 1, 1);

        int idx = static_cast<int>(norm_val * 255.0f);
        idx = std::max(0, std::min(255, idx));

        return intensity_color_table_pcl_[idx];
    }

    // 线程控制
    std::thread render_thread_;
    std::atomic<bool> should_stop_;
    std::mutex data_mutex_;

    // 数据存储
    Pose current_pose_;
    std::deque<Pose> trajectory_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud_;    // 当前帧点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_map_;  // 累积的全局地图

    // 累积参数
    size_t max_accumulated_points_;
    int downsample_rate_;

    // Pangolin相关
    pangolin::OpenGlRenderState s_cam_;
    pangolin::View d_cam_;

    // UI控件
    std::unique_ptr<pangolin::Var<float>> menu_point_size_;
    std::unique_ptr<pangolin::Var<int>> menu_trajectory_length_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_trajectory_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_current_cloud_;
    std::unique_ptr<pangolin::Var<bool>> menu_show_accumulated_map_;
    std::unique_ptr<pangolin::Var<bool>> menu_background_dark_;
    std::unique_ptr<pangolin::Var<bool>> menu_auto_follow_;
    std::unique_ptr<pangolin::Var<bool>> menu_point_color_mode_;
    std::unique_ptr<pangolin::Var<int>> menu_max_map_points_;
    std::unique_ptr<pangolin::Var<bool>> menu_clear_map_;

    // 颜色表
    std::vector<Eigen::Vector4d> intensity_color_table_pcl_;

    // 静态颜色定义
    static constexpr float trajectory_color_[3] = {1.0f, 0.0f, 0.0f};
    static constexpr float current_cloud_color_[3] = {1.0f, 1.0f, 0.0f};
    static constexpr float local_map_color_[3] = {0.5f, 0.5f, 0.5f};
    static constexpr float pose_color_[3] = {0.0f, 1.0f, 0.0f};
};

#endif  // PANGOLIN_VISUALIZER_H