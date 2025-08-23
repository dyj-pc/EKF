#include "armor_detector/Tracker.h"
#include <Eigen/Dense>
#include <iostream> // 用于打印错误信息

Tracker::Tracker(double dt, MotionModelType model, const std::vector<double>& p0_diag)
    : dt_(dt), model_(model)
{
    // 1) 从传入的向量动态构建初始协方差矩阵 P0
    EKF_t::MatrixXX P0 = EKF_t::MatrixXX::Identity(); // 默认值
    if (p0_diag.size() == N_x) {
        Eigen::DiagonalMatrix<double, N_x> p_diag;
        p_diag.diagonal() = Eigen::Map<const Eigen::Matrix<double, N_x, 1>>(p0_diag.data());
        P0 = p_diag.toDenseMatrix();
    } else {
        // 如果YAML配置中的p0_diagonal尺寸不正确，打印错误并使用单位矩阵
        std::cerr << "[Tracker ERROR] p0_diagonal size is " << p0_diag.size() 
                  << ", but should be " << N_x << ". Using Identity matrix as P0." << std::endl;
    }

    // 2) 改进的 Q 更新函数 (过程噪声)
    // 采用“积分噪声传播法”，考虑位移和速度间的耦合
    EKF_t::UpdateQFunc updateQ = [this]() {
        EKF_t::MatrixXX q;
        q.setZero();

        double t = this->dt_;
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;

        double s2qx = noise_.sigma2_q_x;
        double s2qy = noise_.sigma2_q_y;
        double s2qz = noise_.sigma2_q_z;
        double s2qyaw = noise_.sigma2_q_yaw;

        // --- x ---
        q(0,0) = t4/4 * s2qx; q(0,1) = t3/2 * s2qx;
        q(1,0) = t3/2 * s2qx; q(1,1) = t2   * s2qx;

        // --- y ---
        q(2,2) = t4/4 * s2qy; q(2,3) = t3/2 * s2qy;
        q(3,2) = t3/2 * s2qy; q(3,3) = t2   * s2qy;

        // --- z ---
        q(4,4) = t4/4 * s2qz; q(4,5) = t3/2 * s2qz;
        q(5,4) = t3/2 * s2qz; q(5,5) = t2   * s2qz;

        // -- yaw --
        q(6,6) = t4/4 * s2qyaw; q(6,7) = t3/2 * s2qyaw;
        q(7,6) = t3/2 * s2qyaw; q(7,7) = t2   * s2qyaw;

        return q;
    };

    // 3) 改进的 R 更新函数 (测量噪声)
    // 噪声与测量值相关，更符合相机模型
    EKF_t::UpdateRFunc updateR = [this](const EKF_t::MatrixZ1 &z) {
        EKF_t::MatrixZZ r;
        r.setZero();
        // 测量: [x, y, z, yaw]
        double r_x_val = noise_.r_x * std::max(1.0, std::abs(z[0]));
        double r_y_val = noise_.r_y * std::max(1.0, std::abs(z[1]));
        double r_z_val = noise_.r_z * std::max(1.0, std::abs(z[2]));
        
        r(0,0) = r_x_val;
        r(1,1) = r_y_val;
        r(2,2) = r_z_val;
        r(3,3) = noise_.r_yaw; // yaw 噪声保持固定
        return r;
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
    // 保证角度在 [-pi, pi] 范围内
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