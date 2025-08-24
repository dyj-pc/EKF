#include "armor_detector/Tracker.h"
#include "rclcpp/rclcpp.hpp"

Tracker::Tracker(double dt) : dt_(dt), state(LOST) {
    RobotEKF::UpdateQFunc update_Q = [this]() {
        Eigen::Matrix<double, N_x, N_x> Q;
        double t = this->dt_;
        double s2qx=20.0, s2qy=20.0, s2qz=20.0, s2qyaw=100.0, s2qr=800.0, s2qd_zc=800.0;
        double q_x_x=pow(t,4)/4*s2qx, q_x_vx=pow(t,3)/2*s2qx, q_vx_vx=pow(t,2)*s2qx;
        double q_y_y=pow(t,4)/4*s2qy, q_y_vy=pow(t,3)/2*s2qy, q_vy_vy=pow(t,2)*s2qy;
        double q_z_z=pow(t,4)/4*s2qz, q_z_vz=pow(t,3)/2*s2qz, q_vz_vz=pow(t,2)*s2qz;
        double q_yaw_yaw=pow(t,4)/4*s2qyaw, q_yaw_vyaw=pow(t,3)/2*s2qyaw, q_vyaw_vyaw=pow(t,2)*s2qyaw;
        double q_r=pow(t,2)*s2qr;
        double q_d_zc=pow(t,2)*s2qd_zc;
        Q << q_x_x, q_x_vx, 0, 0, 0, 0, 0, 0, 0, 0,
             q_x_vx, q_vx_vx, 0, 0, 0, 0, 0, 0, 0, 0,
             0, 0, q_y_y, q_y_vy, 0, 0, 0, 0, 0, 0,
             0, 0, q_y_vy, q_vy_vy, 0, 0, 0, 0, 0, 0,
             0, 0, 0, 0, q_z_z, q_z_vz, 0, 0, 0, 0,
             0, 0, 0, 0, q_z_vz, q_vz_vz, 0, 0, 0, 0,
             0, 0, 0, 0, 0, 0, q_yaw_yaw, q_yaw_vyaw, 0, 0,
             0, 0, 0, 0, 0, 0, q_yaw_vyaw, q_vyaw_vyaw, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, q_r, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 0, q_d_zc;
        return Q;
    };
    RobotEKF::UpdateRFunc update_R = [](const Measurement& z) {
        RobotEKF::MatrixZZ R;
        double r_x = 0.05, r_y = 0.05, r_z = 0.05, r_yaw = 0.02;
        R << r_x*std::abs(z[0]), 0, 0, 0,
             0, r_y*std::abs(z[1]), 0, 0,
             0, 0, r_z, 0,
             0, 0, 0, r_yaw;
        return R;
    };
    RobotEKF::MatrixXX P0 = RobotEKF::MatrixXX::Identity();
    P0 = P0 * 10.0;
    ekf_ = std::make_unique<RobotEKF>(Predict(dt_), Measure(), update_Q, update_R, P0);
}

void Tracker::reset(const Measurement& z) {
    state = DETECTING;
    detect_count_ = 0; lost_count_ = 0;
    State x0 = State::Zero();
    double xa = z(0), ya = z(1), za = z(2), yaw = z(3);
    double r = 260.0; // 初始半径先验值 (mm)
    double xc = xa + r * cos(yaw), yc = ya + r * sin(yaw);
    x0(0)=xc; x0(2)=yc; x0(4)=za; x0(6)=yaw; x0(8)=r;
    ekf_->setState(x0);
    RCLCPP_INFO(rclcpp::get_logger("armor_detect_node"), "Tracker RESET!");
}

Tracker::State Tracker::predict() {
    auto state_vec = ekf_->predict();
    if (state == TRACKING) {
        lost_count_++;
        if (lost_count_ > lost_thres_) state = LOST;
        else state = TEMP_LOST;
    } else if (state == DETECTING) {
        lost_count_++;
        if (lost_count_ > lost_thres_ / 2) state = LOST;
    }
    return state_vec;
}

Tracker::State Tracker::update(const Measurement& z) {
    auto state_vec = ekf_->update(z);
    
    // 安全网 #1: 状态量硬性限制 (State Clamping)
    if (state_vec(8) < 120.0) state_vec(8) = 120.0;
    else if (state_vec(8) > 400.0) state_vec(8) = 400.0;
    ekf_->setState(state_vec);

    lost_count_ = 0;
    if (state == DETECTING) {
        detect_count_++;
        if (detect_count_ > tracking_thres_) {
            state = TRACKING;
            RCLCPP_INFO(rclcpp::get_logger("armor_detect_node"), "Tracker stable: TRACKING");
        }
    } else if (state == TEMP_LOST) state = TRACKING;
    return state_vec;
}

Tracker::State Tracker::getTargetState() const { return ekf_->getState(); }

Eigen::Vector3d Tracker::getArmorPosition() const {
    State x = getTargetState();
    double xc=x(0), yc=x(2), zc=x(4), yaw=x(6), r=x(8), d_zc=x(9);
    return {xc - r * cos(yaw), yc - r * sin(yaw), zc + d_zc};
}

Tracker::State Tracker::predictAhead(double t_ahead) const {
    if (t_ahead < 1e-3) return getTargetState();
    State x_k = getTargetState();
    Predict pred(t_ahead);
    State x_final;
    pred(x_k.data(), x_final.data());
    return x_final;
}