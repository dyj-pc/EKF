
#pragma once

#include <string>
#include <Eigen/Dense>
#include "motion_model.hpp"
#include "Tracker.h"

// 用于保存所有可配置参数的结构体
struct AppConfig {
    // 仿真参数
    double dt;
    int duration_frames;
    MotionModelType motion_model;

    // 追踪器参数
    EKF_t::MatrixX1 initial_state;
    double lookahead_time;
    NoiseParams noise_params;
    
    
    
    // 输出路径
    std::string csv_dir;
    std::string plot_dir;
};

// 使用单例模式加载并提供对配置的访问
class Config {
public:
    // 删除拷贝和移动构造函数/赋值运算符
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    // 获取单例实例
    static Config& getInstance();

    // 从 YAML 文件加载配置
    bool load(const std::string& path);

    // 获取已加载的配置
    const AppConfig& get() const;

private:
    // 私有构造函数 (单例模式)
    Config() = default;

    // 将字符串转换为 MotionModelType 的辅助函数
    MotionModelType stringToMotionModel(const std::string& model_str);

    AppConfig params_;
    bool loaded_ = false;
};