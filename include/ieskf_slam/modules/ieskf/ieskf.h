#pragma once
#include <Eigen/Dense>

#include "ieskf_slam/math/SO3.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/imu.h"
namespace IESKFSlam {
class IESKF : private ModuleBase {
   public:
    using Ptr = std::shared_ptr<IESKF>;
    struct State18 {
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d bg;
        Eigen::Vector3d ba;
        Eigen::Vector3d gravity;
        State18() {
            rotation = Eigen::Quaterniond::Identity();
            position = Eigen::Vector3d::Zero();
            velocity = Eigen::Vector3d::Zero();
            bg = Eigen::Vector3d::Zero();
            ba = Eigen::Vector3d::Zero();
            gravity = Eigen::Vector3d::Zero();
        }
    };
    struct State15Inv {
        Eigen::MatrixXd X, theta;
        // Eigen::Quaterniond rotation;
        // Eigen::Vector3d position;
        // Eigen::Vector3d velocity;
        // Eigen::Vector3d bg;
        // Eigen::Vector3d ba;
        // Eigen::Vector3d gravity;
        State15Inv() {
            X = Eigen::MatrixXd::Identity(5, 5);
            theta = Eigen::MatrixXd::Zero(6, 1);
            // rotation = Eigen::Quaterniond::Identity();
            // position = Eigen::Vector3d::Zero();
            // velocity = Eigen::Vector3d::Zero();
            // bg = Eigen::Vector3d::Zero();
            // ba = Eigen::Vector3d::Zero();
            // gravity = Eigen::Vector3d::Zero();
        }
    };
    class CalcZHInterface {
       public:
        virtual bool calculate(const IESKF::State18 &state, Eigen::MatrixXd &z, Eigen::MatrixXd &H) = 0;
    };
    std::shared_ptr<CalcZHInterface> calc_zh_ptr;

   private:
    State18 X;
    State15Inv X_inv;
    Eigen::Matrix<double, 18, 18> P;
    Eigen::Matrix<double, 12, 12> Q;
    int iter_times = 10;

   public:
    IESKF(const std::string &confif_path, const std::string &prefix);
    ~IESKF();
    void predict(IMU &imu, double dt);
    void predictInv(IMU &imut, double dt);
    bool update();
    bool updateInv();
    Eigen::Matrix<double, 18, 1> getErrorState18(const State18 &s1, const State18 &s2);

    const State18 &getX();
    void setX(const State18 &x_in);
};
}  // namespace IESKFSlam