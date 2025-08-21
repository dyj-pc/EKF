cmake_minimum_required(VERSION 3.10)
project(test_EKF)

set(CMAKE_CXX_STANDARD 17)

# include 路径
include_directories(include)

# 依赖 
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp 0.6 REQUIRED) # 查找 yaml-cpp

# Python3 + matplotlib-cpp
find_package(Python3 COMPONENTS Development NumPy REQUIRED)
# 假设你把 matplotlibcpp.h 放在 include/ 目录下
include_directories(${Python3_INCLUDE_DIRS})

# 可执行文件
add_executable(test_EKF 
    src/test.cpp 
    src/Tracker.cpp
    src/config.cpp # 添加新的源文件
)

target_link_libraries(test_EKF PRIVATE 
    Eigen3::Eigen
    ${CERES_LIBRARIES} 
    ${OpenCV_LIBRARIES}
    Python3::Python
    Python3::NumPy
    yaml-cpp   # 链接 yaml-cpp 库
)