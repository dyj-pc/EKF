#include "config.h"
#include <iostream>
#include <yaml-cpp/yaml.h> // 引入 yaml-cpp 头文件

MotionModelType Config::stringToMotionModel(const std::string& model_str) {
    if (model_str == "CONSTANT_VELOCITY") return MotionModelType::CONSTANT_VELOCITY;
    if (model_str == "CONSTANT_ROTATION") return MotionModelType::CONSTANT_ROTATION;
    if (model_str == "CONSTANT_VEL_ROT") return MotionModelType::CONSTANT_VEL_ROT;
    
    // 默认回退选项
    std::cerr << "警告: 未知的运动模型 '" << model_str << "'。将使用默认的 CONSTANT_VEL_ROT。\n";
    return MotionModelType::CONSTANT_VEL_ROT;
}

Config& Config::getInstance() {
    static Config instance;
    return instance;
}

const AppConfig& Config::get() const {
    if (!loaded_) {
        throw std::runtime_error("配置尚未加载。");
    }
    return params_;
}

bool Config::load(const std::string& path) {
    try {
        YAML::Node config = YAML::LoadFile(path);

        // 仿真参数
        params_.dt = config["simulation"]["dt"].as<double>();
        params_.duration_frames = config["simulation"]["duration_frames"].as<int>();
        params_.motion_model = stringToMotionModel(config["simulation"]["motion_model"].as<std::string>());
        
        // 追踪器 - 初始状态
        params_.initial_state = EKF_t::MatrixX1::Zero();
        params_.initial_state[0] = config["tracker"]["initial_state"]["x"].as<double>();
        params_.initial_state[2] = config["tracker"]["initial_state"]["y"].as<double>();
        params_.initial_state[4] = config["tracker"]["initial_state"]["z"].as<double>();
        params_.initial_state[6] = config["tracker"]["initial_state"]["yaw"].as<double>();
        
        // 追踪器 - 前视预测
        params_.lookahead_time = config["tracker"]["lookahead_time"].as<double>();
        
        // 追踪器 - 噪声参数
        params_.noise_params.sigma_acc_x = config["tracker"]["process_noise"]["sigma_acc_x"].as<double>();
        params_.noise_params.sigma_acc_y = config["tracker"]["process_noise"]["sigma_acc_y"].as<double>();
        params_.noise_params.sigma_acc_z = config["tracker"]["process_noise"]["sigma_acc_z"].as<double>();
        params_.noise_params.sigma_acc_yaw = config["tracker"]["process_noise"]["sigma_acc_yaw"].as<double>();
        
        params_.noise_params.sigma_meas_x = config["tracker"]["measurement_noise"]["sigma_meas_x"].as<double>();
        params_.noise_params.sigma_meas_y = config["tracker"]["measurement_noise"]["sigma_meas_y"].as<double>();
        params_.noise_params.sigma_meas_z = config["tracker"]["measurement_noise"]["sigma_meas_z"].as<double>();
        params_.noise_params.sigma_meas_yaw = config["tracker"]["measurement_noise"]["sigma_meas_yaw"].as<double>();

        // 输出路径
        std::string base_dir = config["output"]["base_dir"].as<std::string>();
        params_.csv_dir = base_dir + "/" + config["output"]["csv_subdir"].as<std::string>() + "/";
        params_.plot_dir = base_dir + "/" + config["output"]["plot_subdir"].as<std::string>() + "/";

        loaded_ = true;
        std::cout << "成功从 " << path << " 加载配置" << std::endl;
        return true;

    } catch (const YAML::Exception& e) {
        std::cerr << "加载配置文件失败: " << path << "\n"
                  << "原因: " << e.what() << std::endl;
        return false;
    }
}