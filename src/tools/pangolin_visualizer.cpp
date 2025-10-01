#include "ieskf_slam/tools/pangolin_visualizer.h"

#include <Eigen/src/Core/Matrix.h>
#include <pangolin/pangolin.h>
#include <pcl/common/transforms.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <mutex>
#include <thread>

// 静态颜色初始化
constexpr float PangolinVisualizer::trajectory_color_[3];
constexpr float PangolinVisualizer::current_cloud_color_[3];
constexpr float PangolinVisualizer::local_map_color_[3];
constexpr float PangolinVisualizer::pose_color_[3];

// Pose结构体
PangolinVisualizer::Pose::Pose() : position(Eigen::Vector3d::Zero()), rotation(Eigen::Quaterniond::Identity()) {}

PangolinVisualizer::Pose::Pose(const Eigen::Vector3d& pos, const Eigen::Quaterniond& rot)
    : position(pos), rotation(rot) {}

// 构造析构
PangolinVisualizer::PangolinVisualizer()
    : should_stop_(false),
      current_cloud_(new pcl::PointCloud<pcl::PointXYZI>),
      accumulated_map_(new pcl::PointCloud<pcl::PointXYZI>),
      max_accumulated_points_(1000000),  // 默认最大累积100万个点
      downsample_rate_(1) {}             // 默认不降采样

PangolinVisualizer::~PangolinVisualizer() { stop(); }

// 启动渲染线程
void PangolinVisualizer::start() { render_thread_ = std::thread(&PangolinVisualizer::renderLoop, this); }

// 停止渲染线程
void PangolinVisualizer::stop() {
    should_stop_ = true;
    if (render_thread_.joinable()) {
        render_thread_.join();
    }
}

// 设置累积地图参数
void PangolinVisualizer::setAccumulationParams(size_t max_points, int downsample_rate) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    max_accumulated_points_ = max_points;
    downsample_rate_ = std::max(1, downsample_rate);  // 至少为1
}

// 清除累积的地图
void PangolinVisualizer::clearAccumulatedMap() {
    std::lock_guard<std::mutex> lock(data_mutex_);
    accumulated_map_->clear();
}

Eigen::Matrix4d getFollowCamPose(const PangolinVisualizer::Pose& robot_pose) {
    // 机器人位姿
    Eigen::Vector3d pos = robot_pose.position;
    Eigen::Matrix3d rot = robot_pose.rotation.toRotationMatrix();

    // 定义相机相对机器人坐标系的位置，例如在机器人后方2米，高度1米
    Eigen::Vector3d cam_offset(-2.0, 0.0, 1.0);

    // 计算相机世界坐标 = 机器人位置 + 旋转 * 偏移
    Eigen::Vector3d cam_pos = pos + rot * cam_offset;

    // 相机目标点就是机器人当前的位置
    Eigen::Vector3d target = pos;

    // 定义相机的上方向（一般用世界坐标系的Z轴）
    Eigen::Vector3d up(0, 0, 1);

    // 使用类似 gluLookAt 实现，构造4x4视图矩阵
    Eigen::Vector3d z_axis = (cam_pos - target).normalized();  // 视线方向
    Eigen::Vector3d x_axis = up.cross(z_axis).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix4d view = Eigen::Matrix4d::Identity();

    view(0, 0) = x_axis.x();
    view(0, 1) = x_axis.y();
    view(0, 2) = x_axis.z();
    view(1, 0) = y_axis.x();
    view(1, 1) = y_axis.y();
    view(1, 2) = y_axis.z();
    view(2, 0) = z_axis.x();
    view(2, 1) = z_axis.y();
    view(2, 2) = z_axis.z();

    view(0, 3) = -x_axis.dot(cam_pos);
    view(1, 3) = -y_axis.dot(cam_pos);
    view(2, 3) = -z_axis.dot(cam_pos);

    return view;
}

// 更新位姿（保持原有功能）
void PangolinVisualizer::updatePose(const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_pose_ = Pose(position, rotation);
    trajectory_.push_back(current_pose_);
    if (trajectory_.size() > static_cast<size_t>(*menu_trajectory_length_)) {
        trajectory_.pop_front();
    }
}

// 新增：更新当前时刻的点云和位姿，并累积到全局地图
void PangolinVisualizer::updateCurrentScan(const pcl::PointCloud<pcl::PointXYZI>& current_scan,
                                           const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 更新当前位姿和轨迹
    current_pose_ = Pose(position, rotation);
    trajectory_.push_back(current_pose_);
    if (trajectory_.size() > static_cast<size_t>(*menu_trajectory_length_)) {
        trajectory_.pop_front();
    }

    // 保存当前帧点云（用于显示当前帧，可选）
    *current_cloud_ = current_scan;

    // 将当前帧点云转换到世界坐标系并累积
    accumulatePointCloud(current_scan, position, rotation);
}

// 内部函数：累积点云到全局地图
void PangolinVisualizer::accumulatePointCloud(const pcl::PointCloud<pcl::PointXYZI>& scan,
                                              const Eigen::Vector3d& position, const Eigen::Quaterniond& rotation) {
    if (scan.empty())
        return;

    pcl::PointCloud<pcl::PointXYZI> processed_scan = scan;

    // 降采样处理（如果需要）
    if (downsample_rate_ > 1) {
        pcl::PointCloud<pcl::PointXYZI> downsampled_scan;
        for (size_t i = 0; i < processed_scan.size(); i += downsample_rate_) {
            downsampled_scan.push_back(processed_scan[i]);
        }
        processed_scan = std::move(downsampled_scan);
    }

    // 累积到全局地图
    *accumulated_map_ += processed_scan;

    // 如果累积点数过多，进行简单的降采样（每隔一定数量保留一个点）
    if (accumulated_map_->size() > max_accumulated_points_) {
        pcl::PointCloud<pcl::PointXYZI> downsampled_map;
        size_t step = accumulated_map_->size() / max_accumulated_points_ + 1;

        for (size_t i = 0; i < accumulated_map_->size(); i += step) {
            downsampled_map.push_back((*accumulated_map_)[i]);
        }

        *accumulated_map_ = std::move(downsampled_map);

        std::cout << "Accumulated map downsampled to " << accumulated_map_->size()
                  << " points (max: " << max_accumulated_points_ << ")" << std::endl;
    }
}

void PangolinVisualizer::BuildIntensityTable() {
    intensity_color_table_pcl_.clear();
    intensity_color_table_pcl_.reserve(256);

    for (int i = 0; i < 256; i++) {
        float t = i / 255.0f;

        float r, g, b;

        // Jet colormap算法
        if (t < 0.125f) {
            r = 0.0f;
            g = 0.0f;
            b = 0.5f + t * 4.0f;
        } else if (t < 0.375f) {
            r = 0.0f;
            g = (t - 0.125f) * 4.0f;
            b = 1.0f;
        } else if (t < 0.625f) {
            r = (t - 0.375f) * 4.0f;
            g = 1.0f;
            b = 1.0f - (t - 0.375f) * 4.0f;
        } else if (t < 0.875f) {
            r = 1.0f;
            g = 1.0f - (t - 0.625f) * 4.0f;
            b = 0.0f;
        } else {
            r = 1.0f - (t - 0.875f) * 4.0f;
            g = 0.0f;
            b = 0.0f;
        }

        // 限制范围并降低饱和度
        r = std::max(0.0f, std::min(1.0f, r)) * 0.9f;
        g = std::max(0.0f, std::min(1.0f, g)) * 0.9f;
        b = std::max(0.0f, std::min(1.0f, b)) * 0.9f;

        intensity_color_table_pcl_.emplace_back(r, g, b, 0.8f);
    }
}

// 更新当前点云（保持原有接口兼容性）
void PangolinVisualizer::updateCurrentCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    *current_cloud_ = cloud;
}

// 更新局部地图（保持原有接口兼容性，但现在主要使用累积地图）
void PangolinVisualizer::updateLocalMap(const pcl::PointCloud<pcl::PointXYZI>& map) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    // 这个接口现在可以选择是否使用，因为我们有累积地图了
    // 如果你还想保留这个功能，可以保持原样
    // *local_map_ = map;  // 注释掉，使用累积地图代替
}

// 初始化Pangolin窗口和UI
void PangolinVisualizer::initializePangolin() {
    pangolin::CreateWindowAndBind("IESKF SLAM Visualization", 1920, 1080);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SMOOTH);
    glEnable(GL_LINE_SMOOTH);
    glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

    s_cam_ = pangolin::OpenGlRenderState(
        pangolin::ProjectionMatrix(1920, 1080, 960, 960, 960, 540, 0.1, 5000),
        pangolin::ModelViewLookAt(0, 0, 250,  // 摄像机位置：X=0, Y=0, Z=100，很高的高度
                                  0, 0, 0,    // 观察目标点：原点
                                  1, 0, 0  // 摄像机"上"方向，这里用X轴方向，保证摄像机是"正顶视"（从上往下看）
                                  ));

    // 左侧300像素菜单面板
    pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(300));

    // 右侧3D视图，自适应窗口宽高变化
    d_cam_ = pangolin::CreateDisplay()
                 .SetBounds(0.0, 1.0, pangolin::Attach::Pix(200), 1.0, -1200.0f / 800.0f)
                 .SetHandler(new pangolin::Handler3D(s_cam_));

    // UI控件
    menu_point_size_ = std::make_unique<pangolin::Var<float>>("menu.Point Size", 1.0f, 1.0f, 10.0f);
    menu_trajectory_length_ = std::make_unique<pangolin::Var<int>>("menu.Trajectory Length", 100000, 100, 100000);
    menu_show_trajectory_ = std::make_unique<pangolin::Var<bool>>("menu.Show Trajectory", true, true);
    menu_show_current_cloud_ = std::make_unique<pangolin::Var<bool>>("menu.Show Current Scan", true, true);
    menu_show_accumulated_map_ = std::make_unique<pangolin::Var<bool>>("menu.Show Accumulated Map", true, true);
    // menu_background_dark_ = std::make_unique<pangolin::Var<bool>>("menu.Dark Background", false, false);
    menu_auto_follow_ = std::make_unique<pangolin::Var<bool>>("menu.Auto Follow", true, true);

    // 新增点云颜色模式切换 0 = Z轴, 1 = Intensity
    menu_background_dark_ =
        std::make_unique<pangolin::Var<bool>>("menu.Dark Background", true, true);  // 默认true（暗色背景）
    menu_point_color_mode_ =
        std::make_unique<pangolin::Var<bool>>("menu.Intensity Mode", false, true);  // 默认false（Z轴模式）

    // 新增累积地图控制
    menu_max_map_points_ = std::make_unique<pangolin::Var<int>>(
        "menu.Max Map Points", static_cast<int>(max_accumulated_points_), 100000, 5000000);
    menu_clear_map_ = std::make_unique<pangolin::Var<bool>>("menu.Clear Map", false, false);

    BuildIntensityTable();
}

pangolin::OpenGlMatrix PoseToOpenGlMatrix(const PangolinVisualizer::Pose& pose) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    mat.block<3, 3>(0, 0) = pose.rotation.toRotationMatrix();
    mat.block<3, 1>(0, 3) = pose.position;

    pangolin::OpenGlMatrix ogl_mat;
    // Pangolin是列主序的4x4矩阵
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            ogl_mat.m[c * 4 + r] = static_cast<float>(mat(r, c));
        }
    }
    return ogl_mat;
}

// 渲染循环
void PangolinVisualizer::renderLoop() {
    initializePangolin();

    // 用户交互状态和计时器
    bool user_interacting_ = false;
    auto last_interaction_time = std::chrono::steady_clock::now();
    auto interaction_timeout = std::chrono::seconds(1);

    while (!pangolin::ShouldQuit() && !should_stop_) {
        // 处理用户交互检测
        auto now = std::chrono::steady_clock::now();
        if (user_interacting_ && now - last_interaction_time > interaction_timeout) {
            user_interacting_ = false;
        }

        if (*menu_background_dark_) {
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        } else {
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // 动态设置视角
        {
            std::lock_guard<std::mutex> lock(data_mutex_);

            // 处理清除地图请求
            if (*menu_clear_map_) {
                accumulated_map_->clear();
                *menu_clear_map_ = false;  // 重置按钮状态
                std::cout << "Accumulated map cleared!" << std::endl;
            }

            // 更新最大点数设置
            if (static_cast<size_t>(*menu_max_map_points_) != max_accumulated_points_) {
                max_accumulated_points_ = static_cast<size_t>(*menu_max_map_points_);
                std::cout << "Max accumulated points updated to: " << max_accumulated_points_ << std::endl;
            }

            if (*menu_auto_follow_ && !user_interacting_ && !trajectory_.empty()) {
                Pose temp = current_pose_;
                temp.rotation.setIdentity();
                s_cam_.Follow(PoseToOpenGlMatrix(temp));
            }

            d_cam_.Activate(s_cam_);

            auto bounds = d_cam_.GetBounds();
            int w = static_cast<int>(bounds.w);
            int h = static_cast<int>(bounds.h);
            glViewport(static_cast<int>(bounds.l), static_cast<int>(bounds.b), w, h);

            if (*menu_show_trajectory_)
                drawTrajectory();
            if (*menu_show_current_cloud_ && !current_cloud_->empty()) {
                // 绘制当前帧点云（用不同颜色区分）
                drawCurrentScanHighlighted(*current_cloud_);
            }
            if (*menu_show_accumulated_map_ && !accumulated_map_->empty()) {
                drawPointCloudColorMode(*accumulated_map_);
            }
            drawCurrentPose();
            drawCoordinateFrame();
        }

        pangolin::FinishFrame();

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    pangolin::DestroyWindow("IESKF SLAM Visualization");
}

// 绘制轨迹
void PangolinVisualizer::drawTrajectory() {
    if (trajectory_.size() < 2)
        return;

    glLineWidth(6.0f);
    glColor4f(1.0f, 0.0f, 0.0f, 1.0f);  // 红色
    glBegin(GL_LINE_STRIP);
    for (const auto& pose : trajectory_) {
        glVertex3f(pose.position.x(), pose.position.y(), pose.position.z());
    }
    glEnd();

    glPointSize(8.0f);
    glBegin(GL_POINTS);
    for (const auto& pose : trajectory_) {
        glVertex3f(pose.position.x(), pose.position.y(), pose.position.z());
    }
    glEnd();
}

// 新增：绘制当前帧点云（高亮显示）
void PangolinVisualizer::drawCurrentScanHighlighted(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.empty())
        return;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(*menu_point_size_ * 1.5f);  // 当前帧点更大一些

    glBegin(GL_POINTS);
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        // 当前帧用亮黄色高亮显示
        glColor4f(1.0f, 1.0f, 0.0f, 0.9f);  // 亮黄色，高透明度
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    glDisable(GL_POINT_SPRITE);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);
}

void PangolinVisualizer::drawPointCloudColorMode(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (cloud.empty())
        return;

    // 启用混合模式使透明度生效
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_POINT_SPRITE);
    glEnable(GL_PROGRAM_POINT_SIZE);
    glPointSize(*menu_point_size_);

    float min_intensity = std::numeric_limits<float>::max();
    float max_intensity = std::numeric_limits<float>::lowest();
    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();

    for (const auto& p : cloud.points) {
        if (std::isfinite(p.intensity)) {
            min_intensity = std::min(min_intensity, p.intensity);
            max_intensity = std::max(max_intensity, p.intensity);
        }
        if (std::isfinite(p.z)) {
            min_z = std::min(min_z, p.z);
            max_z = std::max(max_z, p.z);
        }
    }

    float intensity_range = max_intensity - min_intensity;
    if (intensity_range < 1e-6)
        intensity_range = 1.0f;
    float z_range = max_z - min_z;
    if (z_range < 1e-6)
        z_range = 1.0f;

    glBegin(GL_POINTS);
    for (const auto& p : cloud.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
            continue;

        float norm_val = 0.f;
        if (*menu_point_color_mode_ == false) {
            // Z轴模式
            norm_val = (p.z - min_z) / z_range;
            norm_val = std::pow(norm_val, 0.8f);
        } else if (*menu_point_color_mode_ == true) {
            // Intensity模式
            norm_val = (p.intensity - min_intensity) / intensity_range;
        }

        norm_val = std::max(0.0f, std::min(1.0f, norm_val));

        Eigen::Vector4d c = IntensityToRgbPCL(norm_val);
        glColor4f(c.x(), c.y(), c.z(), c.w() * 0.6f);  // 降低累积地图透明度
        glVertex3f(p.x, p.y, p.z);
    }
    glEnd();

    glDisable(GL_POINT_SPRITE);
    glDisable(GL_PROGRAM_POINT_SIZE);
    glDisable(GL_BLEND);
}

// 绘制当前位姿
void PangolinVisualizer::drawCurrentPose() {
    if (trajectory_.empty())
        return;

    const Pose& pose = current_pose_;
    Eigen::Vector3d pos = pose.position;
    Eigen::Matrix3d rot = pose.rotation.toRotationMatrix();

    glLineWidth(4.0f);

    // X轴红色
    glColor4f(1.0f, 0.0f, 0.0f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d x_end = pos + rot.col(0) * 0.8;
    glVertex3f(x_end.x(), x_end.y(), x_end.z());
    glEnd();

    // Y轴绿色
    glColor4f(0.0f, 1.0f, 0.0f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d y_end = pos + rot.col(1) * 0.8;
    glVertex3f(y_end.x(), y_end.y(), y_end.z());
    glEnd();

    // Z轴蓝色
    glColor4f(0.0f, 0.0f, 1.0f, 0.9f);
    glBegin(GL_LINES);
    glVertex3f(pos.x(), pos.y(), pos.z());
    Eigen::Vector3d z_end = pos + rot.col(2) * 0.8;
    glVertex3f(z_end.x(), z_end.y(), z_end.z());
    glEnd();

    // 画一个小立方体表示机器人
    glColor4f(pose_color_[0], pose_color_[1], pose_color_[2], 0.9f);
    glPushMatrix();
    glTranslatef(pos.x(), pos.y(), pos.z());

    Eigen::AngleAxisd aa(pose.rotation);
    Eigen::Vector3d axis = aa.axis();
    double angle = aa.angle() * 180.0 / M_PI;
    glRotatef(static_cast<float>(angle), static_cast<float>(axis.x()), static_cast<float>(axis.y()),
              static_cast<float>(axis.z()));

    pangolin::glDrawColouredCube(-0.15, 0.15);
    glPopMatrix();
}

// 绘制世界坐标系
void PangolinVisualizer::drawCoordinateFrame() {
    glLineWidth(3.0f);

    // X红色
    glColor4f(1.0f, 0.0f, 0.0f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(2, 0, 0);
    glEnd();

    // Y绿色
    glColor4f(0.0f, 1.0f, 0.0f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 2, 0);
    glEnd();

    // Z蓝色
    glColor4f(0.0f, 0.0f, 1.0f, 0.8f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 2);
    glEnd();
}