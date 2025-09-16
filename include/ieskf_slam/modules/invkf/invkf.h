#pragma once
#include <Eigen/Dense>


#include "ieskf_slam/math/SO3.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/imu.h"
namespace IESKFSlam {
    class INVKF : private ModuleBase {
    public:
        using Ptr = std::shared_ptr<INVKF>;

        struct State18{
            Eigen::Quaterniond rotation;
            Eigen::Vector3d position;
            Eigen::Vector3d velocity;
            Eigen::Vector3d bg;
            Eigen::Vector3d ba;
            Eigen::Vector3d gravity;
            State18(){
                rotation = Eigen::Quaterniond::Identity();
                position = Eigen::Vector3d::Zero();
                velocity = Eigen::Vector3d::Zero();
                bg = Eigen::Vector3d::Zero();
                ba = Eigen::Vector3d::Zero();
                gravity = Eigen::Vector3d::Zero();
            }
            Eigen::Matrix<double, 5, 5> matrix() const {
            Eigen::Matrix<double, 5, 5> X = Eigen::Matrix<double, 5, 5>::Identity();

            Eigen::Matrix3d R = rotation.toRotationMatrix();

            // 左上角填 R
            X.block<3,3>(0,0) = R;

            // 第1列之外，依次放 velocity 和 position
            X.block<3,1>(0,3) = velocity;
            X.block<3,1>(0,4) = position;

            // 重力不一定直接放进来（取决于你模型设计），
            // 如果你是扩展状态，可以单独在 filter 里存 gravity

            return X;
        }
        };
        const Eigen::MatrixXd getX();
        const Eigen::VectorXd getTheta();
        const Eigen::MatrixXd getP();
        const Eigen::Quaterniond getRotation();
        const Eigen::Vector3d getVelocity();
        const Eigen::Vector3d getPosition();
        const Eigen::Vector3d getGyroscopeBias();
        const Eigen::Vector3d getAccelerometerBias();
        const Eigen::Vector3d getGravity();
        void setX(const Eigen::MatrixXd& X);
        void setP(const Eigen::MatrixXd& P);
        void setTheta(const Eigen::VectorXd& Theta);
        void setRotation(const Eigen::Matrix3d& R);
        void setVelocity(const Eigen::Vector3d& v);
        void setPosition(const Eigen::Vector3d& p);
        void setGyroscopeBias(const Eigen::Vector3d& bg);
        void setAccelerometerBias(const Eigen::Vector3d& ba);
        void setGravity(const Eigen::Vector3d& g);
        Eigen::MatrixXd getErrorState(const Eigen::MatrixXd &s1, const Eigen::MatrixXd &s2);
        Eigen::VectorXd getErrorTheta(const Eigen::VectorXd &s1, const Eigen::VectorXd &s2);
        INVKF(const std::string &confif_path, const std::string &prefix);
        ~INVKF();
        void predict(IMU &imu, double dt);
        bool update();
        class CalcZHInterface{
            public:
            virtual bool calculate(const INVKF::State18 &state, Eigen::MatrixXd &z, Eigen::MatrixXd &H) = 0;
        };
        std::shared_ptr<CalcZHInterface> calc_zh_ptr;
        
    private:
        State18 state_;
        Eigen::MatrixXd X_;
        Eigen::VectorXd theta_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd Q;
        Eigen::Vector3d gravity;


    };

}  // namespace IESKFSlam