#ifndef MAIN_H
#define MAIN_H

#include <chrono>
#include <memory>
#include <iostream>
#include <functional>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <nlohmann/json.hpp>
#include <vector>
#include <iomanip>
#include <sstream>
#include <string>
#include <array>
#include "xr_msgs/msg/custom.hpp"
#include "xr_msgs/msg/head.hpp"
#include "xr_msgs/msg/controller.hpp"

#include "PXREARobotSDK.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


// Pinocchio 和 CasADi 相关头文件
#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/log.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using json = nlohmann::json;

// IK 求解器类声明
class IKSolver {
public:
    IKSolver(const std::string& urdf_path, const std::string& ee_frame_name);
    
    bool solve(const Eigen::Matrix4d& target_pose, 
               const Eigen::VectorXd& q_guess,
               Eigen::VectorXd& q_solution,
               const Eigen::VectorXd& q_last = Eigen::VectorXd());
    
private:
    void initialize();
    
    // 辅助函数声明
    casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix);
    casadi::DM eigen_to_dm(const Eigen::VectorXd& vec);
    Eigen::VectorXd dm_to_eigen(const casadi::DM& dm);
    
private:
    std::string urdf_path_;
    std::string ee_frame_name_;
    
    // CasADi 优化相关
    casadi::Opti opti_;
    casadi::MX var_q_;
    casadi::MX var_q_last_;
    casadi::MX param_tf_;
    casadi::MX q_ref_;
    casadi::MX joint_w_;
};

// 辅助函数声明
std::vector<double> DMToStdVector(const casadi::DM& dm);
casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix);
std::function<void(void* context, PXREAClientCallbackType type, int status, void* userData)> g_callback;
std::mutex g_callback_mutex;
void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData);
void print_json(const json& j, int indent=1);
std::vector<float> stringToFloatVector(const std::string& input);

// XRNode 类声明
class XRNode : public rclcpp::Node {
public:
    XRNode();
    ~XRNode();

    void lPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void rPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void lJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void rJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishJointState(const Eigen::VectorXd& q, bool is_left);
    void SendLResetArmGoal();
    void SendRResetArmGoal();
    void SendExtendArmGoal();
    void SendRetractArmGoal();
    void EnableArmServo(bool enable);
    // void EnableArmCompliance(bool enable);
    void publishGripperState(double gripper_value, bool is_left);
    void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData);

private:
    rclcpp::Publisher<xr_msgs::msg::Custom>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr l_gripper_joint_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr r_gripper_joint_publisher_;
    bool left_gripper = true;   // 默认开
    bool right_gripper = true;  // 默认开
    bool l_ctl_init = false;
    bool r_ctl_init = false;
    // 关节消息模板
    sensor_msgs::msg::JointState l_gripper_msg_;
    sensor_msgs::msg::JointState r_gripper_msg_;
    double gripper_open_angle_ = 0.07f;;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr l_real_pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr r_real_pose_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr l_real_joint_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr r_real_joint_subscriber_;
    bool l_real_has_new_joint_ = false; // 获取机器人关节
    bool r_real_has_new_joint_ = false;
    bool l_real_has_new_pose_ = false;  // 获取机器人姿态
    bool r_real_has_new_pose_ = false;
    geometry_msgs::msg::PoseStamped l_ctl_init_pose;    // 手柄初始姿态
    geometry_msgs::msg::PoseStamped r_ctl_init_pose;
    geometry_msgs::msg::PoseStamped l_real_pose;        // 机器人姿态
    geometry_msgs::msg::PoseStamped r_real_pose;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr l_emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr r_emergency_stop_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr l_enable_agx_arm_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr r_enable_agx_arm_client_;


    // IK 求解器相关 - 左右臂独立
    std::shared_ptr<IKSolver> ik_solver_left_;
    std::shared_ptr<IKSolver> ik_solver_right_;
    // 左右臂独立的状态变量
    Eigen::VectorXd last_q_left_;
    Eigen::VectorXd last_q_right_;
    bool is_first_valid_ik_left_ = true;
    bool is_first_valid_ik_right_ = true;
    // 关节消息模板
    sensor_msgs::msg::JointState l_joint_msg_;
    sensor_msgs::msg::JointState r_joint_msg_;
    // 跳变检测
    double joint_change_threshold_ = 0.5; // 跳变阈值（弧度），根据实际机械臂调整
    // 跳变检测函数：如果变化过大返回 true
    bool isJointJump(const Eigen::VectorXd& q_new, const Eigen::VectorXd& q_old, bool is_left);


    // ==== 长按检测相关变量 ====
    int pressed_time = 1;
    bool left_primary_pressed_;
    bool right_primary_pressed_;
    bool dual_primary_pressed_;
    bool left_primary_press_sent_;
    bool right_primary_press_sent_;
    bool dual_primary_press_sent_;
    bool left_secondary_pressed_;
    bool right_secondary_pressed_;
    bool dual_secondary_pressed_;
    bool left_secondary_press_sent_;
    bool right_secondary_press_sent_;
    bool dual_secondary_press_sent_;
    std::chrono::steady_clock::time_point left_primary_press_start_;
    std::chrono::steady_clock::time_point right_primary_press_start_;
    std::chrono::steady_clock::time_point dual_primary_press_start_;
    std::chrono::steady_clock::time_point left_secondary_press_start_;
    std::chrono::steady_clock::time_point right_secondary_press_start_;
    std::chrono::steady_clock::time_point dual_secondary_press_start_;
};

#endif // MAIN_H
