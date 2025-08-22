// Com.cpp
#include "utils/Com.h"


SerialCommunicationClass::SerialCommunicationClass(rclcpp::Node* node, std::function<void(const SerialData&)> serialDataCallback) 
: node(node), serialDataCallback(serialDataCallback), fd_(-1) {
    initializeSerial();
}

SerialCommunicationClass::~SerialCommunicationClass() {
    running = false;
    if (fd_ >= 0) {
        close(fd_);
    }
}
    
void SerialCommunicationClass::initializeSerial() {
    std::string port = findAvailableSerialPort();
    if (port.empty()) {
        RCLCPP_ERROR(node->get_logger(), "No available serial port found!");
        return;
    }

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open port %s: %s", port.c_str(), strerror(errno));
        return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(fd_, &tty) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get serial attributes");
        close(fd_);
        fd_ = -1;
        return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~OPOST;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set serial attributes");
        close(fd_);
        fd_ = -1;
        return;
    }

    tcflush(fd_, TCIOFLUSH);
    RCLCPP_DEBUG(node->get_logger(), "Serial initialized: %s", port.c_str());
}

    // 查找可用的串口
std::string SerialCommunicationClass::findAvailableSerialPort() {
    struct dirent *entry;
    DIR *dp = opendir("/dev/");
    if (dp == nullptr) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open /dev/ directory");
        return "";
    }

    std::string port;
    while ((entry = readdir(dp)) != nullptr) {
        if (strncmp(entry->d_name, "ttyACM", 6) == 0) {  // 匹配ttyACM串口
            std::string candidate_port = "/dev/" + std::string(entry->d_name);
            int fd = open(candidate_port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
            if (fd >= 0) {
                close(fd);  // 串口可用，返回串口名称
                port = candidate_port;
                break;
            }
        }
    }

    closedir(dp);
    return port;
}

bool SerialCommunicationClass::sendData(float pitch_target, float yaw_target) {
    if (fd_ >= 0) {
        //pitch_target = -0.01; // 约 0.01对应30°
        //yaw_target = 0;
        // 传入参数使用弧度制 [-M_PI, M_PI]
        // 总大小 = 帧头(2) + 命令码(1) + 长度(1) + pitch_target(4) + yaw_target(2) + CRC(1) = 11字节
        std::array<uint8_t, 11> tx_data{};
        
        tx_data[0] = FRAME_HEADER1;
        tx_data[1] = FRAME_HEADER2;
        tx_data[2] = COMMAND_CODE;
        tx_data[3] = 0x06;  // 数据长度为6（4字节pitch_target + 2字节yaw_target）
        
        // 处理pitch_target (4字节)
        pitch_target = pitch_target *180/M_PI * 0.01/30;
        memcpy(&tx_data[4], &pitch_target, sizeof(float));  // 4字节float
        
        // 处理yaw_target (2字节)
        int16_t yaw_int16 = static_cast<int16_t>(yaw_target * 4096 / M_PI);  // 将float转换为定点数
        while (yaw_int16 > 4095) {
            yaw_int16 -= 8192;
        }
        while (yaw_int16 < -4096) {
            yaw_int16 += 8192;
        }

        //yaw_int16 = 1234;

        memcpy(&tx_data[8], &yaw_int16, sizeof(int16_t));  // 2字节
        
        // 计算并添加CRC
        tx_data[10] = CRC8_Check_Sum(tx_data.data(), 10);

        ssize_t written = write(fd_, tx_data.data(), tx_data.size());
        if (written == static_cast<ssize_t>(tx_data.size())) {
            RCLCPP_DEBUG(node->get_logger(), "TX: pitch_target=%.2f yaw_target=%.2f(int16=%d)", 
                        pitch_target, yaw_target, yaw_int16);
            return true;
        } else {
            RCLCPP_DEBUG(node->get_logger(), "TX write failed: written %ld bytes", 
                        written);
        }
    }
    return false;
}

void SerialCommunicationClass::processFrame(const uint8_t* data) {
    DataFrame frame{};
    size_t offset = 4;

    memcpy(&frame.bullet_velocity, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&frame.bullet_angle, &data[offset], sizeof(float));
    offset += sizeof(float);
    memcpy(&frame.gimbal_yaw, &data[offset], sizeof(int16_t));
    offset += sizeof(int16_t);
    memcpy(&frame.mark, &data[offset], sizeof(uint16_t));
    offset += sizeof(uint16_t);
    memcpy(&frame.color, &data[offset], sizeof(uint8_t));
    offset += sizeof(uint8_t);
    memcpy(&frame.z_rotation_velocity, &data[offset], sizeof(float));

    // 格式化输出
    RCLCPP_DEBUG(node->get_logger(), 
        "\033[1;34m[Received Data]\033[0m\n"
        "\033[1;32mBullet Velocity:\033[0m %.2f m/s\n"
        "\033[1;32mBullet Angle:\033[0m %.2f\n"
        "\033[1;33mGimbal Yaw:\033[0m %d (%.2f°)\n"
        "\033[1;36mMark:\033[0m %d\n"
        "\033[1;31mColor:\033[0m %d\n"
        "\033[1;35mZ Rotation Velocity:\033[0m %.2f rad/s",
        frame.bullet_velocity,
        frame.bullet_angle,
        frame.gimbal_yaw, frame.gimbal_yaw * 180.0 / 4096.0,
        frame.mark,
        frame.color,
        frame.z_rotation_velocity
    );

    SerialData msg;
    msg.bullet_velocity = frame.bullet_velocity;
    msg.bullet_angle = frame.bullet_angle;
    msg.gimbal_yaw = frame.gimbal_yaw;
    msg.color = frame.color;
    
    serialDataCallback(msg);
}

void SerialCommunicationClass::processBuffer() {
    
    // 每次处理最多处理10个帧，防止处理过多数据导致阻塞
    static const size_t MAX_FRAMES_PER_LOOP = 10;
    size_t frames_processed = 0;

    while (buffer_index_ >= FRAME_MIN_SIZE && frames_processed < MAX_FRAMES_PER_LOOP) {
        // 安全检查：如果缓冲区接近满，立即清空
        if (buffer_index_ >= BUFFER_SIZE - 128) {
            RCLCPP_WARN(node->get_logger(), "Buffer approaching capacity (%zu bytes), clearing", buffer_index_);
            buffer_index_ = 0;
            return;
        }

        // 查找帧头
        size_t header_pos = 0;
        bool found_header = false;
        
        // 只在合理范围内查找帧头
        while (header_pos <= buffer_index_ - 3 && header_pos < 128) {
            if (buffer_[header_pos] == FRAME_HEADER1 && 
                buffer_[header_pos + 1] == FRAME_HEADER2 && 
                buffer_[header_pos + 2] == COMMAND_CODE) {
                found_header = true;
                break;
            }
            ++header_pos;
        }

        if (!found_header) {
            // 如果找不到帧头，清空缓冲区
            buffer_index_ = 0;
            return;
        }

        // 如果帧头前有无效数据，移除它们
        if (header_pos > 0) {
            if (header_pos < buffer_index_) {
                memmove(buffer_.data(), buffer_.data() + header_pos, buffer_index_ - header_pos);
                buffer_index_ -= header_pos;
            } else {
                buffer_index_ = 0;
                return;
            }
        }

        // 检查是否有完整的帧
        if (buffer_index_ < 4) {
            return;  // 等待更多数据
        }

        uint8_t data_length = buffer_[3];
        size_t frame_length = data_length + 5;

        // 验证帧长度的合理性
        if (data_length > 64 || frame_length > BUFFER_SIZE) {  // 假设最大帧长度为64字节
            RCLCPP_ERROR(node->get_logger(), "Invalid frame length detected: %zu", frame_length);
            buffer_index_ = 0;
            return;
        }

        if (buffer_index_ < frame_length) {
            return;  // 等待完整帧
        }

        // CRC校验
        if (CRC8_Check_Sum(buffer_.data(), frame_length - 1) == buffer_[frame_length - 1]) {
            processFrame(buffer_.data());
            frames_processed++;
        } else {
            // CRC错误，移除这一帧
            RCLCPP_WARN(node->get_logger(), "CRC check failed, discarding frame");
            memmove(buffer_.data(), buffer_.data() + 3, buffer_index_ - 3);
            buffer_index_ -= 3;
            continue;
        }

        // 移除已处理的帧
        if (frame_length < buffer_index_) {
            memmove(buffer_.data(), buffer_.data() + frame_length, buffer_index_ - frame_length);
            buffer_index_ -= frame_length;
        } else {
            buffer_index_ = 0;
        }
    }

    // 如果还有数据未处理，在下一个循环继续处理
    if (buffer_index_ >= FRAME_MIN_SIZE) {
        RCLCPP_DEBUG(node->get_logger(), "Remaining data in buffer: %zu bytes", buffer_index_);
    }
}

void SerialCommunicationClass::timerCallback() {
    // 检查串口状态
    if (fd_ < 0) {
        if (error_print_slower % 1000 == 0) {
            RCLCPP_ERROR(node->get_logger(), "Serial port not available");
        }
        error_print_slower += 1;
        return;
    }

    // 读取串口数据
    if (buffer_index_ < BUFFER_SIZE - 128) {
        uint8_t temp_buffer[128];
        ssize_t bytes_read = read(fd_, temp_buffer, sizeof(temp_buffer));
        
        if (bytes_read > 0) {
            if (buffer_index_ + bytes_read < BUFFER_SIZE) {
                memcpy(buffer_.data() + buffer_index_, temp_buffer, bytes_read);
                buffer_index_ += bytes_read;
                processBuffer();
            } else {
                RCLCPP_WARN(node->get_logger(), "Buffer near full, discarding data");
                buffer_index_ = 0;
            }
        }
    }
}

void SerialCommunicationClass::timerThread() {
    while (running) {
        auto start = std::chrono::steady_clock::now();

        timerCallback();

        // 休眠至下一次调用
        std::this_thread::sleep_until(start + std::chrono::microseconds(1000));  // 大约1ms周期
    }
}