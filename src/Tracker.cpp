#include "Tracker.h"
#include <Eigen/Dense>

Tracker::Tracker(double dt, MotionModelType model)
    : dt_(dt), model_(model)
{
    // 1) 初始协方差矩阵 P0（按状态分量精细化）
    EKF_t::MatrixXX P0 = EKF_t::MatrixXX::Zero();
    // 建议值，可按项目实际修改（单位：位置 m，速度 m/s，角度 rad，角速度 rad/s）
    const double s_pos_xy = 5.0;   // xy 初始位置不确定性
    const double s_pos_z  = 5.0;   // z 初始位置不确定性
    const double s_vel_xy = 10.0;  // xy 初始速度不确定性
    const double s_vel_z  = 5.0;   // z 初始速度不确定性
    const double s_yaw    = 0.1;   // yaw 初始不确定性
    const double s_yaw_d  = 0.2;   // yaw_rate 初始不确定性

    P0(0,0) = s_pos_xy * s_pos_xy;   // x
    P0(2,2) = s_pos_xy * s_pos_xy;   // y
    P0(4,4) = s_pos_z  * s_pos_z;    // z
    P0(1,1) = s_vel_xy * s_vel_xy;   // vx
    P0(3,3) = s_vel_xy * s_vel_xy;   // vy
    P0(5,5) = s_vel_z  * s_vel_z;    // vz
    P0(6,6) = s_yaw    * s_yaw;      // yaw
    P0(7,7) = s_yaw_d  * s_yaw_d;    // yaw_rate

    // 2) 定义 Q 更新函数（WNA 离散化，按轴分块）
    EKF_t::UpdateQFunc updateQ = [this]() {
        EKF_t::MatrixXX Q = EKF_t::MatrixXX::Zero();

        auto Q2 = [this](double sigma_a) {
            Eigen::Matrix2d M;
            const double dt  = this->dt_;
            const double dt2 = dt * dt;
            const double dt3 = dt2 * dt;
            const double dt4 = dt2 * dt2;
            M << dt4/4.0, dt3/2.0,
                 dt3/2.0, dt2;
            return (sigma_a * sigma_a) * M;
        };

        // x: (0,1), y: (2,3), z: (4,5), yaw: (6,7)
        Q.block<2,2>(0,0) = Q2(noise_.sigma_acc_x);
        Q.block<2,2>(2,2) = Q2(noise_.sigma_acc_y);
        Q.block<2,2>(4,4) = Q2(noise_.sigma_acc_z);
        Q.block<2,2>(6,6) = Q2(noise_.sigma_acc_yaw);
        return Q;
    };

    // 3) 定义 R 更新函数（各向异性测量噪声）
    EKF_t::UpdateRFunc updateR = [this](const EKF_t::MatrixZ1 & /*z*/) {
        EKF_t::MatrixZZ R = EKF_t::MatrixZZ::Zero();
        R(0,0) = noise_.sigma_meas_x   * noise_.sigma_meas_x;
        R(1,1) = noise_.sigma_meas_y   * noise_.sigma_meas_y;
        R(2,2) = noise_.sigma_meas_z   * noise_.sigma_meas_z;
        R(3,3) = noise_.sigma_meas_yaw * noise_.sigma_meas_yaw;
        return R;
    };

    // 4) 构建 EKF
    ekf_ = std::make_unique<EKF_t>(
        Predict(dt_, model_),
        Measure(),
        updateQ,
        updateR,
        P0
    );
}

void Tracker::init(const EKF_t::MatrixX1 &x0) {
    state_ = x0;
    ekf_->setState(x0);
}

EKF_t::MatrixX1 Tracker::predict() {
    state_ = ekf_->predict();
    // 可选：保证角度归一化
    state_[6] = wrapAngle(state_[6]);
    return state_;
}

EKF_t::MatrixX1 Tracker::update(const EKF_t::MatrixZ1 &z) {
    state_ = ekf_->update(z);
    // 可选：保证角度归一化
    state_[6] = wrapAngle(state_[6]);
    return state_;
}

// ========= 前视预测与多步预测（不改变 EKF 内部状态） =========

EKF_t::MatrixX1 Tracker::propagateOnce(const EKF_t::MatrixX1& x_in, double dt_step) const {
    EKF_t::MatrixX1 x_out = x_in;
    // 复用已实现的过程模型 Predict，但不触碰 EKF 内部缓存
    Predict prop(dt_step, model_);
    double x0[N_x], x1[N_x];
    for (int i = 0; i < N_x; ++i) x0[i] = x_out[i];
    prop(x0, x1);
    for (int i = 0; i < N_x; ++i) x_out[i] = x1[i];
    x_out[6] = wrapAngle(x_out[6]);
    return x_out;
}

EKF_t::MatrixX1 Tracker::predictAhead(double t_ahead) const {
    if (t_ahead <= 0.0) return state_;
    // 分步传播，避免一次大步长造成线性化误差
    int steps = static_cast<int>(std::ceil(t_ahead / dt_));
    steps = std::max(1, steps);
    double dt_step = t_ahead / steps;

    EKF_t::MatrixX1 x_pred = state_;
    for (int k = 0; k < steps; ++k) {
        x_pred = propagateOnce(x_pred, dt_step);
    }
    return x_pred;
}

std::vector<EKF_t::MatrixX1> Tracker::predictNSteps(int steps, double dt_step) const {
    std::vector<EKF_t::MatrixX1> traj;
    if (steps <= 0 || dt_step <= 0.0) return traj;
    traj.reserve(steps);
    EKF_t::MatrixX1 x_pred = state_;
    for (int k = 0; k < steps; ++k) {
        x_pred = propagateOnce(x_pred, dt_step);
        traj.push_back(x_pred);
    }
    return traj;
}