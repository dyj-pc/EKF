#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

// ceres
#include <ceres/ceres.h>
// project
#include "extended_kalman_filter.hpp"

enum class MotionModelType {
    CONSTANT_VELOCITY,
    CONSTANT_ROTATION,
    CONSTANT_VEL_ROT
};

constexpr int N_x = 8, N_z = 4; // 状态向量和测量向量的维度

struct Predict {
    explicit Predict(double dt, MotionModelType model = MotionModelType::CONSTANT_VELOCITY) 
        : dt(dt), model(model) {}

    MotionModelType model;
    double dt;

    template <typename T>
    void operator()(const T x0[N_x], T x1[N_x]) const {
        // 状态向量 x0 = [x, vx, y, vy, z, vz, yaw, yaw_rate]
        // 预测下一个状态 x1 = [x', vx', y', vy', z', vz', yaw', yaw_rate']
        for (int i = 0; i < N_x; i++) {
            x1[i] = x0[i];
        }

        // v_xyz
        if (model == MotionModelType::CONSTANT_VELOCITY || model == MotionModelType::CONSTANT_VEL_ROT) {
            // 位置更新
            x1[0] += x0[1] * dt; // x' = x + vx * dt
            x1[2] += x0[3] * dt; // y' = y + vy * dt
            x1[4] += x0[5] * dt; // z' = z + vz * dt
        } 
        else{
            // no velocity update
            x1[1] *= 0.;
            x1[3] *= 0.;
            x1[5] *= 0.;
        }

        // v_yaw
        if (model == MotionModelType::CONSTANT_ROTATION || model == MotionModelType::CONSTANT_VEL_ROT) {
            // 角度更新
            x1[6] += x0[7] * dt; // yaw' = yaw + yaw_rate * dt
        } 
        else {
            // no rotation update
            x1[7] *= 0.;
        }
    }

};

struct Measure {
    template <typename T>
    void operator()(const T x[N_z], T z[N_z]) {
        // 测量模型
        z[0] = x[0];    // x
        z[1] = x[2];    // y
        z[2] = x[4];    // z
        z[3] = x[6];    // yaw
    };
};
using EKF_t = ExtendedKalmanFilter<8, 4, Predict, Measure>;

#endif // ARMOR_SOLVER_MOTION_MODEL_HPP_