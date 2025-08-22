// src/test.cpp
#include <iostream>
#include <iomanip>
#include <fstream>
#include <Eigen/Dense>
#include "motion_model.hpp"
#include "Tracker.h"
#include <random>
#include <chrono>
#include <cstdlib>
#include <vector>
#include <string>

// 新增：matplotlib-cpp
#include "matplotlibcpp.h"
#include "config.h"
namespace plt = matplotlibcpp;

// 生成理想轨迹（用于仿真测量的真值基准）
void simulate_motion(double t, Eigen::Vector4d& z) {
    double x = 200 + 10 * t + 10 * sin(1 * t) + 8 * cos(2 * t);                // 真值 x，添加正弦和余弦项以增加复杂性
    double y = 500 + 20 * t - 2 * t * t ;//+ 10 * sin(0.5 * t);                 // 真值 y
    double z_pos = 100 + 0.5 * t * t + 4 * cos(0.15 * t) + 15 * sin(0.3 * t);   // 真值 z
    double yaw = 0.5 + 0.1 * t + 0.2 * sin(0.5 * t);                         // 真值 yaw

    z(0) = x;
    z(1) = y;
    z(2) = z_pos;
    z(3) = yaw;
}

int main() {
    // ====== 1. 加载配置 ======
    if (!Config::getInstance().load("config.yaml")) {
        return -1;
    }
    const auto& cfg = Config::getInstance().get();

    // ====== 2. 从配置初始化追踪器 ======
    Tracker tracker(cfg.dt, cfg.motion_model);
    tracker.setNoiseParams(cfg.noise_params);
    tracker.init(cfg.initial_state);


    // ====== 3. 从配置设置仿真环境 ======
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine eng(seed);
    std::normal_distribution<double> norm_dist(0.0, 1.0); // 标准正态分布

    // 从配置获取 CSV 和绘图路径
    std::string csv_dir = cfg.csv_dir;
    std::string plot_dir = cfg.plot_dir;
    std::string csv_path = csv_dir + "data.csv";
    std::string csv_traj_path = csv_dir + "multi_step_traj.csv";

    // 确保目录存在
    system(("mkdir -p " + csv_dir).c_str());
    system(("mkdir -p " + plot_dir).c_str());

    std::ofstream ofs(csv_path);
    if (!ofs.is_open()) {
        std::cerr << "无法打开文件 " << csv_path << " 进行写入\n";
        return -1;
    }
    ofs << "frame,time,"
           "meas_x,predicted_x,filtered_x,ahead_x,true_x,"
           "meas_y,predicted_y,filtered_y,ahead_y,true_y,"
           "meas_z,predicted_z,filtered_z,ahead_z,true_z,"
           "meas_yaw,predicted_yaw,filtered_yaw,ahead_yaw,true_yaw\n";

    std::ofstream ofs_traj(csv_traj_path);
    if (ofs_traj.is_open()) {
        ofs_traj << "frame,time,step,dt_step,x,y,z,yaw\n";
    }

     // 控制台输出表头
    std::cout << std::left
              << std::setw(6)  << "frame" << std::setw(8)  << "time"
              << std::setw(14) << "measured_x" << std::setw(14) << "filtered_x" << std::setw(14) << "true_x"
              << std::setw(14) << "measured_y" << std::setw(14) << "filtered_y" << std::setw(14) << "true_y"
              << std::setw(16) << "measured_yaw" << std::setw(16) << "filtered_yaw" << std::setw(16) << "true_yaw"
              << '\n' << std::string(6+8+14*6+16*3, '-') << '\n';

    // 用于绘图的数据向量
    std::vector<double> time_points;
    std::vector<double> meas_x, pred_x, filt_x, ahead_x, true_x;
    std::vector<double> meas_y, pred_y, filt_y, ahead_y, true_y;
    std::vector<double> meas_z, pred_z, filt_z, ahead_z, true_z;
    std::vector<double> meas_yaw, pred_yaw, filt_yaw, ahead_yaw, true_yaw;

    // ====== 4. 使用配置参数进行主循环 ======
    for (int i = 1; i <= cfg.duration_frames; ++i) {
        double t = i * cfg.dt;
        time_points.push_back(t);

        // 生成真值
        Eigen::Vector4d z_true;
        simulate_motion(t, z_true);

        // 改进：根据新模型生成与距离相关的测量噪声
        Eigen::Vector4d z_meas = z_true;
        double std_x = sqrt(cfg.noise_params.r_x * std::max(1.0, std::abs(z_true[0])));
        double std_y = sqrt(cfg.noise_params.r_y * std::max(1.0, std::abs(z_true[1])));
        double std_z = sqrt(cfg.noise_params.r_z * std::max(1.0, std::abs(z_true[2])));
        double std_yaw = sqrt(cfg.noise_params.r_yaw);

        z_meas(0) += norm_dist(eng) * std_x;
        z_meas(1) += norm_dist(eng) * std_y;
        z_meas(2) += norm_dist(eng) * std_z;
        z_meas(3) += norm_dist(eng) * std_yaw;


        // 预测
        tracker.predict();
        auto predicted_state = tracker.getState();

        // 前视预测以补偿延迟
        auto ahead_state = tracker.predictAhead(cfg.lookahead_time);

        // 使用测量值更新
        tracker.update(z_meas);
        auto filtered_state = tracker.getState();

        // 控制台输出
        std::cout << std::right
                  << std::setw(6)  << i
                  << std::setw(8)  << std::fixed << std::setprecision(3) << t
                  << std::setw(14) << std::fixed << std::setprecision(2) << z_meas(0)
                  << std::setw(14) << std::fixed << std::setprecision(2) << filtered_state[0]
                  << std::setw(14) << std::fixed << std::setprecision(2) << z_true[0]
                  << std::setw(14) << std::fixed << std::setprecision(2) << z_meas(1)
                  << std::setw(14) << std::fixed << std::setprecision(2) << filtered_state[2]
                  << std::setw(14) << std::fixed << std::setprecision(2) << z_true[1]
                  << std::setw(16) << std::fixed << std::setprecision(6) << z_meas(3)
                  << std::setw(16) << std::fixed << std::setprecision(6) << filtered_state[6]
                  << std::setw(16) << std::fixed << std::setprecision(6) << z_true[3]
                  << '\n';

        // CSV
        ofs << i << ','
            << std::fixed << std::setprecision(6) << t << ','
            << z_meas(0) << ',' << predicted_state[0] << ',' << filtered_state[0] << ',' << ahead_state[0] << ',' << z_true[0] << ','
            << z_meas(1) << ',' << predicted_state[2] << ',' << filtered_state[2] << ',' << ahead_state[2] << ',' << z_true[1] << ','
            << z_meas(2) << ',' << predicted_state[4] << ',' << filtered_state[4] << ',' << ahead_state[4] << ',' << z_true[2] << ','
            << z_meas(3) << ',' << predicted_state[6] << ',' << filtered_state[6] << ',' << ahead_state[6] << ',' << z_true[3] << '\n';

        // 保存向量用于绘图
        meas_x.push_back(z_meas(0)); pred_x.push_back(predicted_state[0]); filt_x.push_back(filtered_state[0]); ahead_x.push_back(ahead_state[0]); true_x.push_back(z_true[0]);
        meas_y.push_back(z_meas(1)); pred_y.push_back(predicted_state[2]); filt_y.push_back(filtered_state[2]); ahead_y.push_back(ahead_state[2]); true_y.push_back(z_true[1]);
        meas_z.push_back(z_meas(2)); pred_z.push_back(predicted_state[4]); filt_z.push_back(filtered_state[4]); ahead_z.push_back(ahead_state[4]); true_z.push_back(z_true[2]);
        meas_yaw.push_back(z_meas(3)); pred_yaw.push_back(predicted_state[6]); filt_yaw.push_back(filtered_state[6]); ahead_yaw.push_back(ahead_state[6]); true_yaw.push_back(z_true[3]);

        // 可选：多步预测轨迹
        if (ofs_traj.is_open()) {
            int steps = 10;
            auto traj = tracker.predictNSteps(steps, cfg.dt);
            for (int k = 0; k < steps; ++k) {
                ofs_traj << i << ','
                         << std::fixed << std::setprecision(6) << t << ','
                         << (k+1) << ',' << cfg.dt << ','
                         << traj[k][0] << ',' << traj[k][2] << ',' << traj[k][4] << ',' << traj[k][6] << '\n';
            }
        }
    }

    ofs.close();
    if (ofs_traj.is_open()) ofs_traj.close();
    std::cout << "CSV 已保存 -> " << csv_path << "\n";

    // 绘图 Python 脚本 (添加真值曲线)
    std::string python_script = R"(#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def create_comparison_plots(csv_file, output_dir):
    try:
        data = pd.read_csv(csv_file)

        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(20, 14))

        def draw(ax, key_meas, key_pred, key_filt, key_ahead, key_true, title, ylabel):
            ax.plot(data['time'], data[key_true], 'k-',  linewidth=2.5, label='Ground Truth', alpha=0.8)
            ax.plot(data['time'], data[key_meas], 'r.', markersize=4, label='Measured', alpha=0.6)
            # ax.plot(data['time'], data[key_pred], 'g-.', linewidth=1, label='Predicted(t+dt)')
            ax.plot(data['time'], data[key_filt], 'b-',  linewidth=2, label='Filtered')
            ax.plot(data['time'], data[key_ahead], 'm:', linewidth=2, label='Ahead')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True, linestyle='--', alpha=0.4)
            ax.set_title(title, fontsize=14)

        draw(ax1, 'meas_x', 'predicted_x', 'filtered_x', 'ahead_x', 'true_x', 'X Position Comparison', 'X Position (m)')
        draw(ax2, 'meas_y', 'predicted_y', 'filtered_y', 'ahead_y', 'true_y', 'Y Position Comparison', 'Y Position (m)')
        draw(ax3, 'meas_z', 'predicted_z', 'filtered_z', 'ahead_z', 'true_z', 'Z Position Comparison', 'Z Position (m)')
        draw(ax4, 'meas_yaw', 'predicted_yaw', 'filtered_yaw', 'ahead_yaw', 'true_yaw', 'Yaw Angle Comparison', 'Yaw Angle (rad)')

        plt.tight_layout(pad=3.0)
        fig.suptitle('EKF Performance with Improved Noise Model', fontsize=18, fontweight='bold')
        output_path = os.path.join(output_dir, 'ekf_comparison_results.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Combined comparison plot saved to {output_path}")

    except Exception as e:
        print(f"Error creating comparison plots: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python plot_script.py <csv_file> <output_dir>")
        sys.exit(1)

    create_comparison_plots(sys.argv[1], sys.argv[2])
)";

    // 保存并执行 Python 脚本
    std::string py_script_path = plot_dir + "comparison_plot_script.py";
    {
        std::ofstream py_script(py_script_path);
        py_script << python_script;
    }
    std::string chmod_cmd = "chmod +x " + py_script_path;
    system(chmod_cmd.c_str());

    std::string python_cmd = "python3 " + py_script_path + " " + csv_path + " " + plot_dir;
    int ret = system(python_cmd.c_str());
    if (ret == 0) {
        std::cout << "Combined comparison plot created successfully\n";
    } else {
        std::cerr << "Python comparison plotting failed. Please ensure python3, pandas, and matplotlib are installed.\n";
    }

    return 0;
}
