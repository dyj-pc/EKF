#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

#include "extended_kalman_filter.hpp" // 继续使用您的EKF头文件

// 状态维数 N_x = 10, 测量维数 N_z = 4
constexpr int N_x = 10;
constexpr int N_z = 4;

// 状态向量 x: 描述机器人中心的运动
// 0, 1: 机器人中心位置 (xc, yc)
// 2, 3: 机器人中心速度 (v_xc, v_yc)
// 4:    机器人中心高度 (zc)
// 5:    机器人中心垂直速度 (v_zc)
// 6:    机器人Yaw角
// 7:    机器人Yaw角速度
// 8:    机器人半径 (中心到装甲板)
// 9:    装甲板相对中心的高度 (d_zc)

// 测量向量 z: 描述直接观测到的装甲板信息
// 0, 1, 2: 装甲板位置 (xa, ya, za)
// 3:       装甲板Yaw角

struct Predict {
    double dt;
    explicit Predict(double t) : dt(t) {}

    template <typename T>
    void operator()(const T x_in[N_x], T x_out[N_x]) const {
        x_out[0] = x_in[0] + x_in[2] * dt; // xc' = xc + v_xc * dt
        x_out[1] = x_in[1] + x_in[3] * dt; // yc' = yc + v_yc * dt
        x_out[4] = x_in[4] + x_in[5] * dt; // zc' = zc + v_zc * dt
        x_out[6] = x_in[6] + x_in[7] * dt; // yaw' = yaw + v_yaw * dt
        // 速度和几何参数假设为匀速或不变
        x_out[2] = x_in[2];
        x_out[3] = x_in[3];
        x_out[5] = x_in[5];
        x_out[7] = x_in[7];
        x_out[8] = x_in[8];
        x_out[9] = x_in[9];
    }
};

struct Measure {
    template <typename T>
    void operator()(const T x_in[N_x], T z_out[N_z]) const {
        T xc = x_in[0], yc = x_in[1], zc = x_in[4];
        T yaw = x_in[6], r = x_in[8], d_zc = x_in[9];
        z_out[0] = xc - r * ceres::cos(yaw); // xa = xc - r*cos(yaw)
        z_out[1] = yc - r * ceres::sin(yaw); // ya = yc - r*sin(yaw)
        z_out[2] = zc + d_zc;                 // za = zc + d_zc
        z_out[3] = yaw;                       // yaw_a = yaw
    }
};

using RobotEKF = ExtendedKalmanFilter<N_x, N_z, Predict, Measure>;

#endif // ARMOR_SOLVER_MOTION_MODEL_HPP_