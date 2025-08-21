#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include "motion_model.hpp"   // 包含 Predict / Measure / EKF_t 定义

struct NoiseParams {
    // 连续域“白噪声加速度”标准差（单位：m/s^2）与“白噪声角加速度”标准差（rad/s^2）
    double sigma_acc_x   = 2.0;   // x 轴加速度噪声
    double sigma_acc_y   = 2.0;   // y 轴加速度噪声
    double sigma_acc_z   = 1.0;   // z 轴加速度噪声
    double sigma_acc_yaw = 0.2;   // yaw 角加速度噪声

    // 测量噪声标准差（单位：m / rad）
    double sigma_meas_x   = 5.0;
    double sigma_meas_y   = 5.0;
    double sigma_meas_z   = 5.0;
    double sigma_meas_yaw = 0.05; // 与 test.cpp 中的噪声水平相符（约 0.05 rad）
};

class Tracker {
public:
    explicit Tracker(double dt, MotionModelType model = MotionModelType::CONSTANT_VELOCITY);

    void init(const EKF_t::MatrixX1 &x0);

    // 预测下一时刻状态（标准 EKF predict，推进到 t+dt）
    EKF_t::MatrixX1 predict();

    // 基于观测更新状态
    EKF_t::MatrixX1 update(const EKF_t::MatrixZ1 &z);

    // 获取当前状态（滤波后/最近一次操作后的状态）
    EKF_t::MatrixX1 getState() const { return state_; }

    Eigen::Vector3d getPosition() const { return {state_[0], state_[2], state_[4]}; }
    double getYaw() const { return state_[6]; }

    // 设置/获取噪声参数（运行时可调）
    void setNoiseParams(const NoiseParams& np) { noise_ = np; }
    NoiseParams getNoiseParams() const { return noise_; }

    // 提前预测（不改变 EKF 内部状态），常用于补偿时延或减少位置滞后
    EKF_t::MatrixX1 predictAhead(double t_ahead) const;

    // 多步预测（不改变 EKF 内部状态），返回 steps 个未来状态（每步 dt_step）
    std::vector<EKF_t::MatrixX1> predictNSteps(int steps, double dt_step) const;

private:
    static inline double wrapAngle(double a) {
        return std::atan2(std::sin(a), std::cos(a));
    }

    // 对任意时间步长 dt_step 做一次过程传播（不改变内部 EKF 状态）
    EKF_t::MatrixX1 propagateOnce(const EKF_t::MatrixX1& x_in, double dt_step) const;

private:
    double dt_;
    MotionModelType model_;
    std::unique_ptr<EKF_t> ekf_;
    EKF_t::MatrixX1 state_;
    NoiseParams noise_;
};