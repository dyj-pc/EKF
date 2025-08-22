#pragma once
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include <cmath>
#include "motion_model.hpp"   // 包含 Predict / Measure / EKF_t 定义

// 改进后的噪声参数结构体
struct NoiseParams {
    // 过程噪声 Q 的参数 (连续时间白噪声谱密度)
    double sigma2_q_x   = 25.0;  // (m/s^2)^2
    double sigma2_q_y   = 25.0;  // (m/s^2)^2
    double sigma2_q_z   = 10.0;  // (m/s^2)^2
    double sigma2_q_yaw = 1.0;   // (rad/s^2)^2

    // 测量噪声 R 的参数
    double r_x   = 0.005; // 观测噪声系数 x
    double r_y   = 0.005; // 观测噪声系数 y
    double r_z   = 0.005; // 观测噪声系数 z
    double r_yaw = 0.0025; // 观测噪声方差 yaw (rad^2)
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