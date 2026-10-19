cmake_minimum_required(VERSION 3.22)
project(ik_solver LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
SET(CMAKE_BUILD_TYPE "Release")

# ============ 1. 查找依赖 ============
# ROS 2 基础与消息包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(xr_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

# 第三方库：nlohmann_json
find_package(nlohmann_json REQUIRED)

# 运动学与数学库
find_package(pinocchio 3.8.0 REQUIRED)
find_package(Eigen3 3.4 REQUIRED)
find_package(octomap REQUIRED)
find_package(hpp-fcl REQUIRED)

# 查找 casadi 和 qpOASES 库（无标准 CMake 配置）
find_library(CASADI_LIBRARY NAMES casadi PATHS /usr/local/lib /usr/lib)
find_library(QPOASES_LIBRARY NAMES qpOASES PATHS /usr/local/lib /usr/lib)

# ============ 2. 配置 Robot SDK ============
set(ROBOT_SDK "/opt/apps/roboticsservice/SDK")
if(EXISTS ${ROBOT_SDK})
  # message(WARNING "SDK") 
  set(PICORobotSDK_INCLUDE ${ROBOT_SDK}/include)
  set(PICORobotSDK_LIB_PATH ${ROBOT_SDK}/linux/64)
  set(PICORobotSDK_LIB_NAME PXREARobotSDK)
endif()

# 使用 find_library 精确定位 SDK 库文件
find_library(PICO_ROBOT_SDK_LIB
    NAMES ${PICORobotSDK_LIB_NAME}
    PATHS ${PICORobotSDK_LIB_PATH}
    NO_DEFAULT_PATH
)

if(NOT PICO_ROBOT_SDK_LIB)
    message(FATAL_ERROR "PICO Robot SDK library (${PICORobotSDK_LIB_NAME}) not found in ${PICORobotSDK_LIB_PATH}")
endif()

# ============ 3. 可执行文件 ============
add_executable(${PROJECT_NAME} src/main.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# ============ 4. 使用 ament_target_dependencies ============
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  ament_index_cpp
  xr_msgs
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  tf2
)

# ============ 5. 链接其他库 ============
target_link_libraries(${PROJECT_NAME}
    nlohmann_json::nlohmann_json
    pinocchio::pinocchio
    ${OCTOMAP_LIBRARIES}
    hpp-fcl::hpp-fcl
    Eigen3::Eigen
    ${CASADI_LIBRARY}
    ${QPOASES_LIBRARY}
    ${PICO_ROBOT_SDK_LIB}
)

# ============ 6. 包含目录 ============
target_include_directories(${PROJECT_NAME} PRIVATE
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/src
    ${pinocchio_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${HPP_FCL_INCLUDE_DIRS}
    /usr/local/include
    ${PICORobotSDK_INCLUDE}
)

# 在 add_executable 之后，install 之前添加
set_target_properties(ik_solver PROPERTIES
    INSTALL_RPATH "/opt/apps/roboticsservice/SDK/linux/64"
    BUILD_RPATH "/opt/apps/roboticsservice/SDK/linux/64"
)

# ============ 7. 安装 ============
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# ============ 8. 导出依赖 ============
ament_export_dependencies(
    rclcpp
    ament_index_cpp
    xr_msgs
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    tf2
    nlohmann_json
    pinocchio
    Eigen3
    octomap
    hpp-fcl
)

ament_package()
