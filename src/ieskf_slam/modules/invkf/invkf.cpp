#include "ieskf_slam/modules/invkf/invkf.h"
#include "ieskf_slam/math/SEn3.h"
#include "ieskf_slam/math/SE3.h"
#include "ieskf_slam/math/SO3.h"
#include <Eigen/src/Core/Matrix.h>
using namespace liepp;
namespace IESKFSlam{
    INVKF::INVKF(const std::string & config_path,const std::string &prefix):ModuleBase(config_path,prefix,"INVKF"){
        X_ = Eigen::MatrixXd::Identity(5,5);
        theta_ = Eigen::MatrixXd::Zero(6,1);
        P_ = Eigen::MatrixXd::Identity(15,15);
        //Q = Eigen::MatrixXd::Zero(15,15);
        double cov_gyroscope,cov_acceleration,cov_bias_acceleration,cov_bias_gyroscope;
        P_(9,9)   = P_(10,10) = P_(11,11) = 0.0001;
        P_(12,12) = P_(13,13) = P_(14,14) = 0.001;
        P_(15,15) = P_(16,16) = P_(17,17) = 0.00001; 
        readParam("cov_gyroscope",cov_gyroscope,0.1);
        readParam("cov_acceleration",cov_acceleration,0.1);
        readParam("cov_bias_acceleration",cov_bias_acceleration,0.1);
        readParam("cov_bias_gyroscope",cov_bias_gyroscope,0.1);
        Q = Eigen::MatrixXd::Zero(15,15);
        Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d{cov_gyroscope,cov_gyroscope,cov_gyroscope};
        Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d{cov_acceleration,cov_acceleration,cov_acceleration};
        Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d{cov_bias_gyroscope,cov_bias_gyroscope,cov_bias_gyroscope};
        Q.block<3, 3>(12, 12).diagonal() = Eigen::Vector3d{cov_bias_acceleration,cov_bias_acceleration,cov_bias_acceleration};
        state_.ba.setZero();
        state_.bg.setZero();
        state_.gravity.setZero();
        state_.position.setZero();
        state_.rotation.setIdentity();
        state_.velocity.setZero();
        gravity.setZero();
    }
    INVKF::~INVKF(){

    }
    void INVKF::predict(IMU &imu, double dt){
        Eigen::Vector3d gyroscope = imu.gyroscope - state_.bg;
        Eigen::Vector3d acceleration = imu.acceleration - state_.ba;
        auto rotation = state_.rotation;
        auto velocity = state_.velocity;
        auto position = state_.position;
        // Eigen::Matrix3d rotation = X_.block(0,0,3,3);
        // Eigen::Vector3d velocity = X_.block(0,3,3,1);
        // Eigen::Vector3d position = X_.block(0,4,3,1);

        Eigen::Vector3d phi = gyroscope * dt;
        Eigen::Matrix3d temp = rotation.toRotationMatrix();
        Eigen::Quaterniond rotation_pred = Eigen::Quaterniond(temp * SO3<double>::exp(phi).asMatrix());
        Eigen::Vector3d velocity_pred = velocity + (rotation * acceleration + gravity) * dt;
        Eigen::Vector3d position_pred = position + velocity * dt;
        //  + 0.5 * (rotation * acceleration + gravity) * dt * dt
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(15,15);
       

        state_.rotation = rotation_pred;
        state_.velocity = velocity_pred;
        state_.position = position_pred;
        // setRotation(rotation_pred);
        // setVelocity(velocity_pred);
        // setPosition(position_pred);
        A.block(3,0,3,3) = SO3<double>::skew(gravity) * dt;
        A.block(6,3,3,3) = Eigen::Matrix3d::Identity();
        A.block(0,9,3,3) = rotation.toRotationMatrix() * (-1) * dt;
        A.block(3,9,3,3) = -1 * SO3<double>::skew(velocity) * rotation * dt;
        A.block(3,12,3,3) = -1 * rotation.toRotationMatrix() * dt;
        A.block(6,9,3,3) = -1 * SO3<double>::skew(position) * rotation * dt;

        Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(15,15) + A;
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(15,15);
        // Adj.block(0,0,3,3) = rotation.toRotationMatrix();
        // Adj.block(3,3,3,3) = rotation.toRotationMatrix();
        // Adj.block(6,6,3,3) = rotation.toRotationMatrix();
        // Adj.block(3,0,3,3) = liepp::SO3<double>::skew(velocity) * rotation;
        // Adj.block(6,0,3,3) = liepp::SO3<double>::skew(position) * rotation;
        Adj.block(0,0,9,9) = SEn3<2,double>::Adjoint_SEK3(state_.matrix());//!!!!!!!!!!!!!!!!11
        
        Eigen::MatrixXd PhiAdj = Phi * Adj;//std::cout << "5"  << std::endl;
        Eigen::MatrixXd Q_hat = PhiAdj * Q * PhiAdj.transpose() * dt;
        P_ = Phi * P_ * Phi.transpose() + Q_hat;
    }
    bool INVKF::update(){
        auto x = state_;
        // Eigen::Matrix <double,5,5> x = X_;
        Eigen::MatrixXd K;
        Eigen::MatrixXd H;
        Eigen::MatrixXd z;
        // state_.position = state_.position + Eigen::Vector3d(0,0,1);
        calc_zh_ptr->calculate(x,z,H);
        Eigen::MatrixXd H_t = H.transpose();
        K = (H_t*H+(P_/0.001).inverse()).inverse()*H_t; //公式18
        Eigen::VectorXd delta = K * z ;
            // 李群指数更新部分
        Eigen::MatrixXd dX = SEn3<2,double>::Exp_SEK3(delta.segment(0,9));
        Eigen::MatrixXd X = state_.matrix(); // 把 state 转成整体矩阵 [R,v,p]
        X = dX * X;

        // 从更新后的矩阵 X 里解码回 state
        state_.rotation = X.block<3,3>(0,0);
        state_.rotation.normalize();
        state_.velocity = X.block<3,1>(0,3);
        state_.position = X.block<3,1>(0,4);

        // bias 更新
        Eigen::VectorXd dTheta = delta.segment(9, 6);
        state_.bg += dTheta.head(3);
        state_.ba += dTheta.tail(3);

        P_ = (Eigen::Matrix<double,15,15>::Identity()-K*H)*P_;

        
        return true;

 
    }

    const Eigen::MatrixXd INVKF::getP(){return P_;}
    const Eigen::Quaterniond INVKF::getRotation(){return state_.rotation;}
    const Eigen::Vector3d INVKF::getVelocity(){return state_.velocity;}
    const Eigen::Vector3d INVKF::getPosition(){return state_.position;}
    const Eigen::Vector3d INVKF::getGyroscopeBias(){return state_.bg;}
    const Eigen::Vector3d INVKF::getAccelerometerBias(){return state_.ba;}
    const Eigen::Vector3d INVKF::getGravity(){return gravity;}

    void INVKF::setP(const Eigen::MatrixXd& P){P_ = P;}

    // void INVKF::setRotation(const Eigen::Matrix3d& R){X_.block(0,0,3,3) = R;}
    // void INVKF::setVelocity(const Eigen::Vector3d& v){X_.block(0,3,3,1) = v;}
    // // void INVKF::setPosition(const Eigen::Vector3d& p){X_.block(0,4,3,1) = p;}
    void INVKF::setGyroscopeBias(const Eigen::Vector3d& bg){state_.bg = bg;}
    // void INVKF::setAccelerometerBias(const Eigen::Vector3d& ba){theta_.tail(3) = ba;}
    void INVKF::setGravity(const Eigen::Vector3d& g){gravity = g;}
    Eigen::VectorXd INVKF::getErrorTheta(const Eigen::VectorXd &s1, const Eigen::VectorXd &s2){return s1-s2;}
    Eigen::MatrixXd INVKF::getErrorState(const Eigen::MatrixXd &s1, const Eigen::MatrixXd &s2){
        return s1*s2.inverse();
    }
}