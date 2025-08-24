// ArmorDetect_Node.cpp
#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include "camera/Camera.h"
#include "armor_detector/LightBarDetector.h"
#include "armor_detector/ArmorDetector.h"
#include "armor_detector/ArmorClassifier.h"
#include "armor_detector/ArmorSolver.h"
#include "armor_detector/ArmorAngleKalman.h"

#include "armor_detector/Tracker.h"
#include <angles/angles.h>

//#include "auto_aim/msg/serial_data.hpp"
//#include "auto_aim/msg/gimbal_command.hpp"
#include <chrono>
#include <string>
#include <thread>
#include <armor_detector/BallisticSolver.h>
#include <yaml-cpp/yaml.h>
#include "utils/FrameRateCounter.h"
#include "utils/UnwarpUtils.h"
#include "test_codes/VideoInput.h"
#include "test_codes/ImagesInput.h"
#include <iostream>
#include <sstream>
#include <filesystem>
#include <unistd.h>
#include <limits.h>
#include <queue>
#include "utils/Com.h"
#include <csignal>
#include "test_codes/PredictionTrans.h"

namespace fs = std::filesystem;


#define USE_VIDEO // 定义后使用视频而不是摄像头作为输入
//#define USE_IMAGES // 定义后使用图片而不是摄像头作为输入
//#define SAVE_IMG_FREQ 30 // 定义后将每n帧保存一次相机图片

// 全局变量定义
cv::Mat g_image;
pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
bool g_bExit = false;
bool image_used = true;

class ArmorDetectNode : public rclcpp::Node {
public:
    ArmorDetectNode() : Node("armor_detect_node") {

        // 1. 获取可执行文件路径
        char exec_path[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", exec_path, sizeof(exec_path) - 1);
        if (len == -1) {
            perror("readlink");
            return;
        }
        exec_path[len] = '\0';
        RCLCPP_INFO(this->get_logger(), "info from C++ | Path: %s\n", exec_path);
        // 2. 转换为文件系统路径对象
        fs::path full_path(exec_path);
        std::string full_path_str = full_path.string();  // 转换为字符串便于查找
        // 3. 查找工作空间目录名
        const std::string ws_dir_name = "transistor_rm2026_algorithm_visual_ws";
        size_t pos = full_path_str.find(ws_dir_name);
        if (pos == std::string::npos) {
            std::cerr << "Error: Workspace directory not found in path" << std::endl;
            return;
        }
        // 4. 截取到工作空间目录结尾
        fs::path ws_dir_path = full_path_str.substr(0, pos + ws_dir_name.length());
        // 5. 拼接模型路径
        const std::string config_file_relatvie_path = "src/shared_files/config.yaml";
        fs::path config_file_path = ws_dir_path / config_file_relatvie_path;  // 使用文件系统的路径拼接

        // 加载配置文件
        config_file_ptr = std::make_shared<YAML::Node>(YAML::LoadFile(config_file_path));



        // 初始化串口通信器
        serial_communication_ = std::make_shared<SerialCommunicationClass>(this, std::bind(&ArmorDetectNode::serialDataCallback, this, std::placeholders::_1));

        // 初始化参数
        bullet_velocity_ = (*config_file_ptr)["bullet_velocity_"].as<float>();
        current_pitch_ = (*config_file_ptr)["current_pitch_"].as<float>();
        current_yaw_ = (*config_file_ptr)["current_yaw_"].as<float>();

        delta_x_ = (*config_file_ptr)["delta_x_"].as<float>();
        delta_y_ = (*config_file_ptr)["delta_y_"].as<float>();
        delta_z_ = (*config_file_ptr)["delta_z_"].as<float>();

        RESET_DISTANCE_THRESHOLD = (*config_file_ptr)["RESET_DISTANCE_THRESHOLD"].as<float>(); 
        MAX_LOST_TIME = (*config_file_ptr)["MAX_LOST_TIME"].as<float>(); 

        has_valid_target_ = false;
        enemy_color_ = (*config_file_ptr)["enemy_color"].as<std::string>();

        yaw_rad_to_x_pixel_ratio = (*config_file_ptr)["yaw_rad_to_x_pixel_ratio"].as<float>(); 
        pitch_rad_to_y_pixel_ratio = (*config_file_ptr)["pitch_rad_to_y_pixel_ratio"].as<float>(); 
        
        params_.min_light_height = (*config_file_ptr)["min_light_height"].as<int>();
        params_.light_slope_offset = (*config_file_ptr)["light_slope_offset"].as<int>();
        params_.light_min_area = (*config_file_ptr)["light_min_area"].as<int>();
        params_.max_light_wh_ratio = (*config_file_ptr)["max_light_wh_ratio"].as<float>();
        params_.min_light_wh_ratio = (*config_file_ptr)["min_light_wh_ratio"].as<float>();
        params_.light_max_tilt_angle = (*config_file_ptr)["light_max_tilt_angle"].as<float>();
        params_.min_light_delta_x = (*config_file_ptr)["min_light_delta_x"].as<int>();
        params_.min_light_dx_ratio = (*config_file_ptr)["min_light_dx_ratio"].as<float>();
        params_.max_light_dy_ratio = (*config_file_ptr)["max_light_dy_ratio"].as<float>();
        params_.max_light_delta_angle = (*config_file_ptr)["max_light_delta_angle"].as<float>();
        params_.near_face_v = (*config_file_ptr)["near_face_v"].as<int>();
        params_.max_lr_rate = (*config_file_ptr)["max_lr_rate"].as<float>();
        params_.max_wh_ratio = (*config_file_ptr)["max_wh_ratio"].as<float>();
        params_.min_wh_ratio = (*config_file_ptr)["min_wh_ratio"].as<float>();
        params_.small_armor_wh_threshold = (*config_file_ptr)["small_armor_wh_threshold"].as<float>();
        params_.bin_cls_thres = (*config_file_ptr)["bin_cls_thres"].as<int>();
        params_.target_max_angle = (*config_file_ptr)["target_max_angle"].as<int>();
        params_.goodToTotalRatio = (*config_file_ptr)["goodToTotalRatio"].as<float>();
        params_.matchDistThre = (*config_file_ptr)["matchDistThre"].as<int>();
        params_.wh_ratio_threshold = (*config_file_ptr)["wh_ratio_threshold"].as<float>();
        params_.wh_ratio_max = (*config_file_ptr)["wh_ratio_max"].as<float>();
        params_.M_YAW_THRES = (*config_file_ptr)["M_YAW_THRES"].as<int>();
        params_.K_YAW_THRES = (*config_file_ptr)["K_YAW_THRES"].as<float>();
        params_.MAX_DETECT_CNT = (*config_file_ptr)["MAX_DETECT_CNT"].as<int>();
        params_.MAX_LOST_CNT = (*config_file_ptr)["MAX_LOST_CNT"].as<int>();
        
        frame_rate_ = (*config_file_ptr)["frame_rate"].as<float>();
        // ========== 新的 EKF 和 Tracker 初始化 ==========
        double dt = 1.0 / std::max(1.0f, frame_rate_);
        tracker_ = std::make_unique<Tracker>(dt);
        RCLCPP_INFO(this->get_logger(), "New 10D EKF Tracker initialized with dt=%.4f", dt);
        // ========== 初始化结束 ==========


#ifdef USE_VIDEO
        video_input_ = std::make_shared<VideoInput>(ws_dir_path / (*config_file_ptr)["video_relative_path"].as<std::string>());
#else
#ifdef USE_IMAGES
        images_input_ = std::make_shared<ImagesInput>(ws_dir_path / (*config_file_ptr)["images_relative_path"].as<std::string>());
#else
        // 初始化相机和检测器
        camera_ = std::make_shared<Camera>((*config_file_ptr)["cam_ip"].as<std::string>(), (*config_file_ptr)["pc_ip"].as<std::string>());
        camera_->setExposureTime((*config_file_ptr)["camera_ExposureTime"].as<float>());
        camera_->setGain((*config_file_ptr)["camera_Gain"].as<float>());
#endif
#endif
        reset_com_frame = (*config_file_ptr)["reset_com_frame"].as<int>();

        if (enemy_color_ == "RED") {
            params_.enemy_color = Params::RED;
        } else if (enemy_color_ == "BLUE") {
            params_.enemy_color = Params::BLUE;
        } else if (enemy_color_ == "GREEN") {
            params_.enemy_color = Params::GREEN;
        } else if (enemy_color_ == "BOTH") {
            params_.enemy_color = Params::BOTH;
        } else {
            // 处理错误情况，设置默认值
            enemy_color_ = "GREEN";
            params_.enemy_color = Params::GREEN;
        }

        light_detector_ = std::make_shared<LightBarDetector>(params_, config_file_ptr, this);
        armor_detector_ = std::make_shared<ArmorDetector>(config_file_ptr, this);
        classifier_ = std::make_shared<ArmorClassifier>(config_file_ptr, this);
        armor_solver_ = std::make_shared<ArmorSolver>(config_file_ptr, this);
        angle_kalman_ = std::make_shared<ArmorAngleKalman>();

        trans_pred_ = std::make_shared<Trans2DPredTo3DClass>(config_file_ptr);

        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000/frame_rate_)), // 33
            std::bind(&ArmorDetectNode::processImage, this));

        com_timer_thread_ = std::thread(std::bind(&SerialCommunicationClass::timerThread, serial_communication_));
        com_timer_thread_.detach();

        fps_counter = std::make_shared<FrameRateCounter>(30); // 30帧滑动窗口统计帧率

        // 串口通信下位机初始化
        serial_communication_->sendData(0, 0);

        RCLCPP_INFO(this->get_logger(), "ArmorDetectNode initialized");
    }

    ~ArmorDetectNode() {
        serial_communication_->~SerialCommunicationClass();
        cv::destroyAllWindows();
        pthread_mutex_destroy(&g_mutex);
        RCLCPP_INFO(this->get_logger(), "ArmorDetectNode destroyed");
    }

private:
    void serialDataCallback(const SerialData& msg) {
        bullet_velocity_ = msg.bullet_velocity;
        current_pitch_ = ((float)(msg.bullet_angle)) * 30 / 1.8 * M_PI / 180; // 测定pitch轴传入数据1.8大约对应30°
        current_yaw_ = ((float)(msg.gimbal_yaw)) * M_PI / 4096.0;  // 一圈对应[-4096, 4095]
        enemy_color_ = (msg.color == 0) ? "RED" : "BLUE";
        if (enemy_color_ == "RED") {
            params_.enemy_color = Params::RED;
        } else if (enemy_color_ == "BLUE") {
            params_.enemy_color = Params::BLUE;
        }
        if (light_detector_) {
            light_detector_->setEnemyColor(msg.color == 0 ? Params::RED : Params::BLUE);
        }

        if (current_yaw_ < -M_PI/2 && last_yaw_rad_ > M_PI/2) {
            yaw_circle_ += 1;
        } else if (current_yaw_ > M_PI/2 && last_yaw_rad_ < -M_PI/2) {
            yaw_circle_ -= 1;
        }
        total_yaw_rad_ = yaw_circle_ * 2 * M_PI + current_yaw_;
        last_pitch_rad_ = current_pitch_;
        last_yaw_rad_ = current_yaw_;

        ground_stable_point = cv::Point2f(2000+total_yaw_rad_*yaw_rad_to_x_pixel_ratio, 500+last_pitch_rad_*pitch_rad_to_y_pixel_ratio);

        RCLCPP_DEBUG(this->get_logger(), 
            "Received serial data: v=%.2f, pitch=%.2f, yaw=%.2f, color=%s \nyaw_circle=%d, total_yaw_rad=%.2f, point=(%.1f, %.1f)",
            bullet_velocity_, current_pitch_, current_yaw_, enemy_color_.c_str(),
            yaw_circle_, total_yaw_rad_, ground_stable_point.x, ground_stable_point.y);
    }

    void drawResults(cv::Mat& image, 
                     const std::vector<Light>& lights,
                     const std::vector<Armor>& armors,
                     const std::vector<ArmorResult>& classifyResults,
                     const std::vector<ArmorResult>& classifyResults_forFourierPredict) {
        cv::Mat result = image.clone();

        // 0. 绘制地面系不动点（DEBUG）
        cv::circle(result, ground_stable_point_delay.front(), 10, cv::Scalar(0, 255, 0), 2);
        /* cv::circle(result, cv::Point2f(1000, 1000) - ground_stable_point_delay.front(), 10, cv::Scalar(0, 255, 0), 2);
        for (const auto& res : classifyResults) {
            for (size_t i = 0; i < res.corners.size() && i < 4; i++) {
                cv::line(result, res.corners[i] - ground_stable_point_delay.front() + cv::Point2f(500, 500), 
                        res.corners[(i+1)%4] - ground_stable_point_delay.front() + cv::Point2f(500, 500), 
                        cv::Scalar(0, 255, 0), 2);
            }    
        } */

        // 1. 绘制灯条（绿色）
        for (const auto& light : lights) {
            cv::Point2f vertices[4];
            light.el.points(vertices);
            for (int i = 0; i < 4; i++) {
                cv::line(result, vertices[i], vertices[(i + 1) % 4], 
                        cv::Scalar(0, 255, 0), 2);
            }
        }

        // 2. 绘制装甲板候选区域（黄色）
        for (const auto& armor : armors) {
            for (size_t i = 0; i < armor.corners.size() && i < 4; i++) {
                cv::line(result, armor.corners[i], 
                        armor.corners[(i+1)%4], 
                        cv::Scalar(0, 255, 255), 2);
            }

            // 显示装甲板置信度
            if (!armor.corners.empty()) {
                std::string conf_str = cv::format("conf: %.2f", armor.confidence);
                cv::Point text_pos(armor.corners[0].x, armor.corners[0].y - 10);
                cv::putText(result, conf_str, text_pos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, 
                        cv::Scalar(0, 255, 255), 1);
            }
        }

        // 3. 绘制最终识别结果（红色）和跟踪信息
        for (const auto& res : classifyResults_forFourierPredict) {
            if (res.is_steady_tracked) {
                for (auto& prediction : res.predictions) {
                    cv::circle(result, prediction, 3, cv::Scalar(0, 255, 0), -1);
                }
            }
        }
        for (const auto& res : classifyResults) {
            // 绘制装甲板轮廓
            if (res.is_tracked_now) {
                for (size_t i = 0; i < res.corners.size() && i < 4; i++) {
                    cv::line(result, res.corners[i], 
                            res.corners[(i+1)%4], 
                            cv::Scalar(0, 0, 255), 2);
                }    
            } else {
                for (size_t i = 0; i < res.corners.size() && i < 4; i++) {
                    cv::line(result, res.corners[i], 
                            res.corners[(i+1)%4], 
                            cv::Scalar(255, 0, 255), 2);
                }    
            }

            // 绘制预测中心点
            for (auto& prediction : res.predictions) {
                cv::circle(result, prediction, 3, cv::Scalar(255, 0, 255), -1);
            }
            cv::circle(result, res.center_predicted, 3, cv::Scalar(0, 255, 255), -1);

            // 绘制中心点和编号
            cv::Point2f center = res.center;
            cv::circle(result, center, 3, cv::Scalar(0, 0, 255), -1);

            std::string text = cv::format("N%d (%.2f)", 
                                        res.number, 
                                        res.confidence);
            cv::Point text_pos(res.corners[1].x, res.corners[1].y - 10);

            // 使用黑色描边使文字更清晰
            cv::putText(result, text, text_pos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cv::Scalar(0, 0, 0), 3);
            cv::putText(result, text, text_pos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, 
                        cv::Scalar(0, 0, 255), 1);

            // 添加跟踪状态显示
            std::string track_text = "TRACKING";
            cv::Point track_pos(center.x - 30, center.y + 30);
            cv::putText(result, track_text, track_pos,
                        cv::FONT_HERSHEY_SIMPLEX, 0.5,
                        cv::Scalar(0, 255, 0), 1);
        }

        // 在窗口中显示图像
        cv::imshow("Armor Detection", result);
        cv::waitKey(1);  // 确保窗口刷新
    }

    void processImage() {
        
        cv::Mat frame;
#ifdef USE_VIDEO
        while (image_used)
        {
            usleep(1000);
        }
#endif
#ifdef USE_IMAGES
        while (image_used)
        {
            usleep(1000);
        }
#endif
        pthread_mutex_lock(&g_mutex);
        if (!g_image.empty()) {
            frame = g_image.clone();
            image_used = true;
        }
        pthread_mutex_unlock(&g_mutex);

        if (!frame.empty()) {
            
#ifdef SAVE_IMG_FREQ
            frame_count_ += 1;
            if (frame_count_ % SAVE_IMG_FREQ == 0 && frame_count_ / SAVE_IMG_FREQ < 2000) {
                // 创建保存目录
                fs::create_directories("camera_images");
                // 生成文件名（00001.jpg 格式）
                std::ostringstream filename;
                filename << "camera_images/"
                        << std::setw(5) << std::setfill('0') << (frame_count_ / SAVE_IMG_FREQ)
                        << ".jpg";
                cv::imwrite(filename.str(), frame);
            }
#endif

            //cv::flip(frame, frame, -1);  // 翻转图像（上下翻转）

            std::vector<Light> lights;
            std::vector<Armor> armors;
            std::vector<ArmorResult> classifyResults;
            std::vector<ArmorResult> classifyResults_forFourierPredict;
            std::vector<std::vector<ArmorResult>> classifyResults_expanded;

            // 检测灯条
            light_detector_->detectLights({frame});
            light_detector_->processLights();
            lights = light_detector_->getLights();
            
            // 检测装甲板
            armors = armor_detector_->detectArmors(lights);
            has_valid_target_ = false;


            ground_stable_point_delay.push(ground_stable_point);
            while (ground_stable_point_delay.size() > 2) {
                ground_stable_point_delay.pop();
            }
            classifyResults_expanded = classifier_->classify(frame, armors, ground_stable_point_delay.front());
            classifyResults = classifyResults_expanded[0];
            classifyResults_forFourierPredict = classifyResults_expanded[1];

            if (armors.empty() && reset_com_frame_count >= reset_com_frame) {
                serial_communication_->sendData(0, 0);
                pitch_integrate_temp = 0; // 积分项重置
            }
            if (reset_com_frame_count < reset_com_frame) {
                reset_com_frame_count += 1;
            }


            if (!armors.empty()) {
                reset_com_frame_count = 0;
                // 选择最佳目标（置信度最高）
                auto it = std::max_element(
                    classifyResults.begin(), classifyResults.end(),
                    [](const ArmorResult& a, const ArmorResult& b) {
                        return a.confidence < b.confidence;
                    }
                );
                if (it != classifyResults.end()) {
                    auto best_result = *it;
                    AimResult aim = armor_solver_->solveArmor(best_result);
                    if (aim.valid) {
                        // 安全网 #2: 连续化Yaw角处理
                        double continuous_yaw = last_continuous_yaw_ + angles::shortest_angular_distance(last_continuous_yaw_, aim.yaw);
                        last_continuous_yaw_ = continuous_yaw;
                        

                        // ========== EKF REFACTORED LOGIC START ==========

                        // 1. 构造4维测量向量 z = [xa, ya, za, yaw_a]
                        Tracker::Measurement z;
                        z << aim.position.x, aim.position.y, aim.position.z, aim.yaw;

                        // 2. EKF 状态机逻辑
                        if (tracker_->state == Tracker::LOST) {
                            // 如果是丢失状态，用当前测量值重置滤波器
                            tracker_->reset(z);
                            current_target_id_ = best_result.number;
                        } else {
                            // 安全网 #3: 智能跳变处理
                            Eigen::Vector3d pred_armor_pos = tracker_->getArmorPosition();
                            double position_diff = (pred_armor_pos - Eigen::Vector3d(aim.position.x, aim.position.y, aim.position.z)).norm();

                            if (best_result.number != current_target_id_ || position_diff > RESET_DISTANCE_THRESHOLD) {
                                if(best_result.number != current_target_id_) {
                                    RCLCPP_WARN(this->get_logger(), "ID switched, resetting tracker.");
                                } else {
                                    RCLCPP_WARN(this->get_logger(), "Position jumped (%.f mm), resetting tracker.", position_diff);
                                }
                                tracker_->reset(z);
                                current_target_id_ = best_result.number;
                            } else {
                                tracker_->predict();
                                tracker_->update(z);
                            }
                        }

                        // 3. 提前预测与弹道解算
                        // 计算总延迟
                        constexpr float image_latency = 0.033f;
                        constexpr float comm_latency  = 0.010f;
                        float bullet_time = (bullet_velocity_ > 1.0f) ? (std::abs(aim.position.z) / 1000.0f / bullet_velocity_) : 0.0f;
                        float total_delay = image_latency + comm_latency + bullet_time;

                        // 获取提前预测后的机器人中心状态
                        Tracker::State future_state = tracker_->predictAhead(total_delay);

                        // 从预测的机器人中心状态，反解出未来时刻装甲板的位置
                        double future_xc = future_state(0), future_yc = future_state(1), future_zc = future_state(4);
                        double future_yaw = future_state(6), future_r = future_state(8), future_d_zc = future_state(9);
                        cv::Point3f predicted_pos(
                            future_xc - future_r * cos(future_yaw),
                            future_yc - future_r * sin(future_yaw),
                            future_zc + future_d_zc
                        );

                        // 弹道解算
                        BallisticInfo ballistic_result = calcBallisticAngle(
                            predicted_pos.x, 
                            predicted_pos.y, 
                            predicted_pos.z,
                            delta_x_,
                            delta_y_,
                            delta_z_,
                            bullet_velocity_,
                            pitch_integrate_temp,//current_pitch_,
                            current_yaw_
                        );
                        
                        if (ballistic_result.valid) {
                            // RCLCPP_INFO(this->get_logger(), "Target detected, publishing command");
                            has_valid_target_ = true;

                            pitch_integrate_temp += ballistic_result.pitch_angle * 0.1;

                            if (pitch_integrate_temp > 0.3) {
                                pitch_integrate_temp = 0.3;
                            }
                            if (pitch_integrate_temp < -0.3) {
                                pitch_integrate_temp = -0.3;
                            }
                            
                            // 发布云台控制命令
                            float command_pitch = last_pitch_rad_ + ballistic_result.pitch_angle * 0.7 + pitch_integrate_temp; // PI控制
                            float command_yaw = ballistic_result.yaw_angle;
                            serial_communication_->sendData(command_pitch, command_yaw);

                            RCLCPP_DEBUG(this->get_logger(),
                                "Target %d: Position[%.2f, %.2f, %.2f] mm, "
                                "Command[pitch: %.2f, yaw: %.2f] rad",
                                best_result.number,
                                predicted_pos.x, predicted_pos.y, predicted_pos.z,
                                command_pitch, command_yaw);
                            
                                // 绘制预测点（黄色）
                                cv::Point2f pred_pixel = armor_solver_->project3DToPixel(predicted_pos);
                                cv::circle(frame, pred_pixel, 8, cv::Scalar(0, 255, 255), 2);
                        }
                    }
                    
                }
            } else {
                if (tracker_->state != Tracker::LOST) {
                    tracker_->predict();
                }
            }

            drawResults(frame, lights, armors, classifyResults, classifyResults_forFourierPredict);

            //计算帧率
            fps_counter->tick();
            
            // // 显示当前参数状态
            // cv::putText(frame, 
            //     cv::format("V: %.1f m/s, P: %.1f, Y: %.1f", 
            //         bullet_velocity_, current_pitch_, current_yaw_),
            //     cv::Point(10, 60),
            //     cv::FONT_HERSHEY_SIMPLEX, 0.5,
            //     cv::Scalar(0, 255, 0), 1);
        }        

        // 获取处理帧率
        RCLCPP_INFO(this->get_logger(), "frame rate: %.1f fps\n" , fps_counter->fps());
    }

    // 参数文件
    std::shared_ptr<YAML::Node> config_file_ptr; 

    // 处理目标丢失情况
    // void handleTargetLost() {
    //     if (!is_target_lost_) {
    //         RCLCPP_WARN(get_logger(), "Target lost!");
    //         is_target_lost_ = true;
    //         last_track_time_ = this->now();
    //     }
    // }
    // 新增成员变量
    int current_target_id_ = -1;      // 当前跟踪目标ID
    cv::Point3f last_observed_pos_;   // 上一帧观测位置
    bool is_target_lost_ = false;     // 目标丢失标志
    rclcpp::Time last_track_time_;    // 最后有效跟踪时间

    // 配置参数
    // static constexpr float RESET_DISTANCE_THRESHOLD = 400.0f; // 单位：mm
    // static constexpr float MAX_LOST_TIME = 0.5f;              // 单位：秒
    float RESET_DISTANCE_THRESHOLD; // 单位：mm
    float MAX_LOST_TIME;              // 单位：秒
    // 成员变量
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread com_timer_thread_;
    
    std::shared_ptr<Camera> camera_;
    std::shared_ptr<LightBarDetector> light_detector_;
    std::shared_ptr<ArmorDetector> armor_detector_;
    std::shared_ptr<ArmorSolver> armor_solver_;
    std::shared_ptr<ArmorClassifier> classifier_;
    std::shared_ptr<ArmorAngleKalman> angle_kalman_;

    std::shared_ptr<VideoInput> video_input_;
    std::shared_ptr<ImagesInput> images_input_;
    float frame_rate_;

    std::shared_ptr<Trans2DPredTo3DClass> trans_pred_;
    
    float bullet_velocity_;
    float current_pitch_;
    float current_yaw_;
    float delta_x_;
    float delta_y_;
    float delta_z_;
    float last_pitch_rad_ = 0;
    float last_yaw_rad_ = 0;
    int yaw_circle_ = 0;
    float total_yaw_rad_ = 0;
    bool has_valid_target_;
    std::string enemy_color_;
    Params params_;

    // EKF/Tracker 相关新增成员
    std::unique_ptr<Tracker> tracker_;
    double last_continuous_yaw_ = 0.0; // 用于连续化yaw角

    // 帧率计算器
    std::shared_ptr<FrameRateCounter> fps_counter;
#ifdef SAVE_IMG_FREQ
    long long frame_count_ = 0;
#endif
    float pitch_integrate_temp = 0.0;
    cv::Point2f ground_stable_point;
    std::queue<cv::Point2f> ground_stable_point_delay;
    std::shared_ptr<SerialCommunicationClass> serial_communication_;
    int reset_com_frame;
    int reset_com_frame_count = 0;
    float yaw_rad_to_x_pixel_ratio;
    float pitch_rad_to_y_pixel_ratio;
};

std::shared_ptr<ArmorDetectNode> node;
void signalHandler(int signum) {
    if (node) {
        rclcpp::shutdown();
    }
}
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    node = std::make_shared<ArmorDetectNode>();
    signal(SIGINT, signalHandler);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
