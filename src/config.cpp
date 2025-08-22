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
        const auto& initialStateNode = config["tracker"]["initial_state"];
        params_.initial_state = EKF_t::MatrixX1::Zero();
        params_.initial_state << initialStateNode[0].as<double>(), initialStateNode[1].as<double>(),
                                 initialStateNode[2].as<double>(), initialStateNode[3].as<double>(),
                                 initialStateNode[4].as<double>(), initialStateNode[5].as<double>(),
                                 initialStateNode[6].as<double>(), initialStateNode[7].as<double>();
        
        // 追踪器 - 前视预测
        params_.lookahead_time = config["tracker"]["lookahead_time"].as<double>();
        
        // 追踪器 - 改进后的噪声参数
        const auto& processNoiseNode = config["tracker"]["process_noise"];
        params_.noise_params.sigma2_q_x = processNoiseNode["sigma2_q_x"].as<double>();
        params_.noise_params.sigma2_q_y = processNoiseNode["sigma2_q_y"].as<double>();
        params_.noise_params.sigma2_q_z = processNoiseNode["sigma2_q_z"].as<double>();
        params_.noise_params.sigma2_q_yaw = processNoiseNode["sigma2_q_yaw"].as<double>();
        
        const auto& measurementNoiseNode = config["tracker"]["measurement_noise"];
        params_.noise_params.r_x = measurementNoiseNode["r_x"].as<double>();
        params_.noise_params.r_y = measurementNoiseNode["r_y"].as<double>();
        params_.noise_params.r_z = measurementNoiseNode["r_z"].as<double>();
        params_.noise_params.r_yaw = measurementNoiseNode["r_yaw"].as<double>();

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
