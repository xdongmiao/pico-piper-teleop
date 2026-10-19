#include "main.h"

// ==================== IKSolver 实现 ====================
IKSolver::IKSolver(const std::string& urdf_path, const std::string& ee_frame_name)
    : urdf_path_(urdf_path), ee_frame_name_(ee_frame_name) {
    initialize();
}

bool IKSolver::solve(const Eigen::Matrix4d& target_pose, 
               const Eigen::VectorXd& q_guess,
               Eigen::VectorXd& q_solution,
               const Eigen::VectorXd& q_last) {
    
    // 设置目标位姿
    casadi::DM Tf_dm = matrix_to_dm(target_pose);
    opti_.set_value(param_tf_, Tf_dm);
    
    // 设置初始猜测
    casadi::DM q_init = eigen_to_dm(q_guess);
    opti_.set_initial(var_q_, q_init);
    
    // 设置上一时刻关节角（用于平滑项）
    if (q_last.size() > 0) {
        casadi::DM q_last_dm = eigen_to_dm(q_last);
        opti_.set_value(var_q_last_, q_last_dm);
    } else {
        opti_.set_value(var_q_last_, q_init);
    }
    
    // 求解
    try {
        casadi::OptiSol sol = opti_.solve();
        casadi::DM q_sol_dm = sol.value(var_q_);
        q_solution = dm_to_eigen(q_sol_dm);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "IK solve error: " << e.what() << std::endl;
        return false;
    }
}

void IKSolver::initialize() {
    // 1. 加载模型（使用 SX 进行符号计算）
    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path_, model);
    
    // 2. 转换为 CasADi 符号模型（SX）
    using ADScalar = casadi::SX;
    pinocchio::ModelTpl<ADScalar> ad_model = model.cast<ADScalar>();
    pinocchio::DataTpl<ADScalar> ad_data(ad_model);
    
    // 3. 定义 CasADi 符号关节变量（SX）
    casadi::SX q_sym = casadi::SX::sym("q", model.nq);
    casadi::SX Tf_sym = casadi::SX::sym("tf", 4, 4);
    
    pinocchio::ModelTpl<ADScalar>::ConfigVectorType q_ad(model.nq);
    pinocchio::casadi::copy(q_sym, q_ad);
    
    // 4. 符号前向运动学
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad);
    pinocchio::updateFramePlacements(ad_model, ad_data);
    
    // 获取末端 frame
    pinocchio::FrameIndex hand_id = ad_model.getFrameId(ee_frame_name_);
    
    // 5. 构建误差函数（SX）
    // 平移误差
    auto tcol = [](const casadi::SX& T4x4) {
        return T4x4(casadi::Slice(0,3), casadi::Slice(3,4));
    };
    
    auto eig3_to_sx = [](const Eigen::Matrix<casadi::SX,3,1>& v) {
        casadi::SX out(3,1);
        for(int i=0;i<3;++i) out(i) = v(i);
        return out;
    };
    
    casadi::SX p = eig3_to_sx(ad_data.oMf[hand_id].translation());
    casadi::SX p_err = p - tcol(Tf_sym);
    
    // 旋转误差（SO(3) 对数映射）
    const Eigen::Matrix<casadi::SX,3,3>& R = ad_data.oMf[hand_id].rotation();
    Eigen::Matrix<casadi::SX,3,3> R_des;
    for(int i = 0; i < 3; ++i) {
        for(int j = 0; j < 3; ++j) {
            R_des(i,j) = Tf_sym(i,j);
        }
    }
    
    Eigen::Matrix<casadi::SX,3,3> Rerr = R_des.transpose() * R;
    Eigen::Matrix<casadi::SX,3,3> skew = 0.5 * (Rerr - Rerr.transpose());
    Eigen::Matrix<casadi::SX,3,1> rot_err;
    rot_err << skew(2,1), skew(0,2), skew(1,0);
    
    casadi::SX rot_err_sx = casadi::SX::vertcat({rot_err(0), rot_err(1), rot_err(2)});
    
    // 6. 创建 CasADi 函数（SX -> SX）
    casadi::Function translational_error_func("translational_error", 
        {q_sym, Tf_sym}, {p_err});
    casadi::Function rotational_error_func("rotational_error", 
        {q_sym, Tf_sym}, {rot_err_sx});
    
    // 7. 构建优化问题（使用 MX）
    opti_ = casadi::Opti();
    
    // 优化变量和参数（MX）
    var_q_ = opti_.variable(model.nq);
    var_q_last_ = opti_.parameter(model.nq);
    param_tf_ = opti_.parameter(4, 4);
    q_ref_ = opti_.parameter(model.nq);
    joint_w_ = opti_.parameter(model.nq);
    
    // 调用 SX 函数，但传入 MX 变量，CasADi 会自动转换
    casadi::MX translational_cost = casadi::MX::sumsqr(
        translational_error_func(casadi::MXVector{var_q_, param_tf_}).at(0));
    casadi::MX rotational_cost = casadi::MX::sumsqr(
        rotational_error_func(casadi::MXVector{var_q_, param_tf_}).at(0));
    
    // 其他代价项
    casadi::MX smooth_cost = casadi::MX::sumsqr(var_q_ - var_q_last_);
    casadi::MX joint_i_cost = casadi::MX::sumsqr(casadi::MX::diag(joint_w_) * (var_q_ - q_ref_));
    
    // 约束：关节限位
    Eigen::VectorXd q_min = model.lowerPositionLimit;
    Eigen::VectorXd q_max = model.upperPositionLimit;
    casadi::DM q_min_dm(q_min.size(), 1);
    casadi::DM q_max_dm(q_max.size(), 1);
    for (int i = 0; i < q_min.size(); ++i) {
        q_min_dm(i) = q_min(i);
        q_max_dm(i) = q_max(i);
    }
    opti_.subject_to(opti_.bounded(q_min_dm, var_q_, q_max_dm));
    
    // 总代价
    opti_.minimize(50.0 * translational_cost + rotational_cost + 0.04 * smooth_cost);// + 0.06 * joint_i_cost);
    
    // 设置参考值和权重（默认值）
    casadi::DM ref_data = casadi::DM::zeros(model.nq, 1);
    opti_.set_value(q_ref_, ref_data);
    
    casadi::DM w_data = casadi::DM::ones(model.nq, 1);
    opti_.set_value(joint_w_, w_data);
    
    // 求解器配置
    casadi::Dict opts;
    opts["expand"] = true;
    opts["detect_simple_bounds"] = true;
    opts["calc_lam_p"] = false;
    opts["print_time"] = false;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.print_level"] = 0;
    opts["ipopt.max_iter"] = 30;
    opts["ipopt.tol"] = 1e-4;
    opts["ipopt.acceptable_tol"] = 5e-4;
    opts["ipopt.acceptable_iter"] = 5;
    opts["ipopt.max_wall_time"] = 1e-2;
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.derivative_test"] = "none";
    opts["ipopt.jacobian_approximation"] = "exact";
    
    opti_.solver("ipopt", opts);
}

casadi::DM IKSolver::matrix_to_dm(const Eigen::Matrix4d& matrix) {
    casadi::DM ret = casadi::DM::eye(4);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            ret(i,j) = matrix(i,j);
        }
    }
    return ret;
}

casadi::DM IKSolver::eigen_to_dm(const Eigen::VectorXd& vec) {
    casadi::DM dm(vec.size(), 1);
    for(int i = 0; i < vec.size(); ++i) {
        dm(i) = vec(i);
    }
    return dm;
}

Eigen::VectorXd IKSolver::dm_to_eigen(const casadi::DM& dm) {
    Eigen::VectorXd vec(dm.size1());
    for(int i = 0; i < dm.size1(); ++i) {
        vec(i) = static_cast<double>(dm(i));
    }
    return vec;
}

// ==================== 辅助函数实现 ====================
std::vector<double> DMToStdVector(const casadi::DM& dm) {
    std::vector<double> v(dm.size1());
    for (int i = 0; i < dm.size1(); ++i)
        v[i] = static_cast<double>(dm(i));
    return v;
}

casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix) {
    casadi::DM ret = casadi::DM::eye(4);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            ret(i,j) = matrix(i,j);
        }
    }
    return ret;
}

void callbackForwarder(void* context, PXREAClientCallbackType type, int status, void* userData) {
    std::lock_guard<std::mutex> lock(g_callback_mutex);
    if (g_callback) {
        g_callback(context, type, status, userData);
    }
}

void print_json(const json& j, int indent) {
    std::string indent_str(indent * 2, ' ');
    if (j.is_object()) {
        std::cout << indent_str << "{\n";
        for (auto it = j.begin(); it != j.end(); ++it) {
            std::cout << indent_str << "  \"" << it.key() << "\": ";
            print_json(it.value(), indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "}";
    } else if (j.is_array()) {
        std::cout << indent_str << "[\n";
        for (const auto& item : j) {
            std::cout << indent_str << "  ";
            print_json(item, indent + 1);
            std::cout << ",\n";
        }
        if (!j.empty()) {
            std::cout << "\b\b" << std::endl;
        }
        std::cout << indent_str << "]";
    } else if (j.is_string()) {
        std::cout << "\"" << j.get<std::string>() << "\"";
    } else if (j.is_boolean()) {
        std::cout << (j.get<bool>() ? "true" : "false");
    } else if (j.is_number_integer()) {
        std::cout << j.get<int64_t>();
    } else if (j.is_number_unsigned()) {
        std::cout << j.get<uint64_t>();
    } else if (j.is_number_float()) {
        std::cout << j.get<double>();
    } else if (j.is_null()) {
        std::cout << "null";
    } else {
        std::cout << "unknown type";
    }
}

std::vector<float> stringToFloatVector(const std::string& input) {
    std::vector<float> result;
    std::stringstream ss(input);
    std::string token;
    while (std::getline(ss, token, ',')) {
        try {
            result.push_back(std::stof(token));
        } catch (const std::exception& e) {
            std::cerr << "转换错误: " << token << " -> " << e.what() << std::endl;
        }
    }
    return result;
}

// ==================== XRNode 实现 ====================
XRNode::XRNode() : Node("pico") {
    publisher_ = this->create_publisher<xr_msgs::msg::Custom>("xr_pose", 10);

    // // sim
    // l_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    // real
    l_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/left/control/move_j", 10);
    r_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/right/control/move_j", 10);
    l_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/left/control/joint_states", 10);
    r_gripper_joint_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/right/control/joint_states", 10);

    l_real_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/left/feedback/tcp_pose", 10, std::bind(&XRNode::lPoseCallback, this, std::placeholders::_1));
    r_real_pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/right/feedback/tcp_pose", 10, std::bind(&XRNode::rPoseCallback, this, std::placeholders::_1));
    l_real_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/left/feedback/joint_states", 10, std::bind(&XRNode::lJointCallback, this, std::placeholders::_1));
    r_real_joint_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/right/feedback/joint_states", 10, std::bind(&XRNode::rJointCallback, this, std::placeholders::_1));

    l_emergency_stop_client_ = this->create_client<std_srvs::srv::Empty>("/left/emergency_stop");
    r_emergency_stop_client_ = this->create_client<std_srvs::srv::Empty>("/right/emergency_stop");
    l_enable_agx_arm_client_ = this->create_client<std_srvs::srv::SetBool>("/left/enable_agx_arm");
    r_enable_agx_arm_client_ = this->create_client<std_srvs::srv::SetBool>("/right/enable_agx_arm");
    
    std::string urdf_path = "/projects/pico_tele_agx/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/piper/urdf/piper_description.urdf";
    std::string ee_frame_name = "joint6";
    ik_solver_left_ = std::make_shared<IKSolver>(urdf_path, ee_frame_name);
    ik_solver_right_ = std::make_shared<IKSolver>(urdf_path, ee_frame_name);
    // Eigen::VectorXd q_home(7);
    // q_home << 0.0, 0.0, 0.0, 1.5707, 0.0, 0, 1.5707;
    last_q_left_ = Eigen::VectorXd::Zero(6);
    last_q_right_ = Eigen::VectorXd::Zero(6);
    // 初始化关节消息模板
    l_joint_msg_.header.frame_id = "base_link";
    l_joint_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    l_joint_msg_.position.resize(6);
    l_joint_msg_.velocity.clear();
    l_joint_msg_.effort.clear();
    r_joint_msg_.header.frame_id = "base_link";
    r_joint_msg_.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    r_joint_msg_.position.resize(6);
    r_joint_msg_.velocity.clear();
    r_joint_msg_.effort.clear();
    l_gripper_msg_.header.frame_id = "base_link";
    l_gripper_msg_.name = {"gripper"};
    l_gripper_msg_.position.resize(1);
    l_gripper_msg_.velocity.clear();
    l_gripper_msg_.effort.clear();
    r_gripper_msg_.header.frame_id = "base_link";
    r_gripper_msg_.name = {"gripper"};
    r_gripper_msg_.position.resize(1);
    r_gripper_msg_.velocity.clear();
    r_gripper_msg_.effort.clear();

    // 初始化按钮状态
    left_primary_pressed_ = left_secondary_pressed_ = false;
    right_primary_pressed_ = right_secondary_pressed_ = false;
    dual_primary_pressed_ = dual_secondary_pressed_ = false;
    left_primary_press_sent_ = left_secondary_press_sent_ = false;
    right_primary_press_sent_ = right_secondary_press_sent_ = false;
    dual_primary_press_sent_ = dual_secondary_press_sent_ = false;
}

XRNode::~XRNode() {}

void XRNode::lPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (l_real_has_new_pose_ == false) {
        l_real_pose.pose.position.x = msg->pose.position.x;
        l_real_pose.pose.position.y = msg->pose.position.y;
        l_real_pose.pose.position.z = msg->pose.position.z;
        l_real_pose.pose.orientation.x = msg->pose.orientation.x;
        l_real_pose.pose.orientation.y = msg->pose.orientation.y;
        l_real_pose.pose.orientation.z = msg->pose.orientation.z;
        l_real_pose.pose.orientation.w = msg->pose.orientation.w;
        l_real_has_new_pose_ = true;

        RCLCPP_INFO(this->get_logger(), "\033[1;33mLeft Real Pose Topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
                    l_real_pose_subscriber_->get_topic_name(),
                    l_real_pose.pose.position.x, l_real_pose.pose.position.y, l_real_pose.pose.position.z,
                    l_real_pose.pose.orientation.x, l_real_pose.pose.orientation.y, 
                    l_real_pose.pose.orientation.z, l_real_pose.pose.orientation.w);
    }
}
void XRNode::rPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (r_real_has_new_pose_ == false) {
        r_real_pose.pose.position.x = msg->pose.position.x;
        r_real_pose.pose.position.y = msg->pose.position.y;
        r_real_pose.pose.position.z = msg->pose.position.z;
        r_real_pose.pose.orientation.x = msg->pose.orientation.x;
        r_real_pose.pose.orientation.y = msg->pose.orientation.y;
        r_real_pose.pose.orientation.z = msg->pose.orientation.z;
        r_real_pose.pose.orientation.w = msg->pose.orientation.w;
        r_real_has_new_pose_ = true;

        RCLCPP_INFO(this->get_logger(), "\033[1;33mRight Real Pose Topic: %s\n- 位置: [x: %.3f, y: %.3f, z: %.3f]\n姿态: [qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f]\033[0m", 
                    r_real_pose_subscriber_->get_topic_name(),
                    r_real_pose.pose.position.x, r_real_pose.pose.position.y, r_real_pose.pose.position.z,
                    r_real_pose.pose.orientation.x, r_real_pose.pose.orientation.y, 
                    r_real_pose.pose.orientation.z, r_real_pose.pose.orientation.w);
    }
}
void XRNode::lJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (l_real_has_new_joint_ == false) {
        l_real_has_new_joint_ = true;

        Eigen::VectorXd q_home(6);
        q_home <<  msg->position[0], msg->position[1], msg->position[2],
                   msg->position[3], msg->position[4], msg->position[5];
        last_q_left_ = q_home;

        RCLCPP_INFO(this->get_logger(), "\033[1;33mLeft Real Joint Topic: %s\n- %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\033[0m", 
                    l_real_joint_subscriber_->get_topic_name(),
                    last_q_left_[0], last_q_left_[1], last_q_left_[2],
                    last_q_left_[3], last_q_left_[4], last_q_left_[5]);
    }
}
void XRNode::rJointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (r_real_has_new_joint_ == false) {
        r_real_has_new_joint_ = true;

        Eigen::VectorXd q_home(6);
        q_home <<  msg->position[0], msg->position[1], msg->position[2],
                   msg->position[3], msg->position[4], msg->position[5];
        last_q_right_ = q_home;

        RCLCPP_INFO(this->get_logger(), "\033[1;33mRight Real Joint Topic: %s\n- %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f\033[0m", 
                    r_real_joint_subscriber_->get_topic_name(),
                    last_q_right_[0], last_q_right_[1], last_q_right_[2],
                    last_q_right_[3], last_q_right_[4], last_q_right_[5]);
    }
}

bool XRNode::isJointJump(const Eigen::VectorXd& q_new, const Eigen::VectorXd& q_old, bool is_left) {
    // 如果是第一次，不检测跳变
    if (is_left) {
        if (is_first_valid_ik_left_) {
            return false;
        }
    } else {
        if (is_first_valid_ik_right_) {
            return false;
        }
    }

    // 遍历所有关节，检查变化量
    for (int i = 0; i < q_new.size(); ++i) {
        double delta = std::abs(q_new(i) - q_old(i));
        if (delta > joint_change_threshold_) {
            RCLCPP_WARN(this->get_logger(), 
                "Joint jump detected at joint %d: delta = %.4f rad (threshold: %.4f)", 
                i, delta, joint_change_threshold_);
            return true;
        }
    }
    return false;
}

void XRNode::publishJointState(const Eigen::VectorXd& q, bool is_left) {
    if (is_left) {
        l_joint_msg_.header.stamp = this->now();
        for (int i = 0; i < 6; ++i) {
            l_joint_msg_.position[i] = q(i);
        }
        l_joint_publisher_->publish(l_joint_msg_);
    } else {
        r_joint_msg_.header.stamp = this->now();
        for (int i = 0; i < 6; ++i) {
            r_joint_msg_.position[i] = q(i);
        }
        r_joint_publisher_->publish(r_joint_msg_);
    }
}
void XRNode::SendLResetArmGoal() {
    l_joint_msg_.header.stamp = this->now();
    l_joint_msg_.position = {0.0, 0.268, -0.75, 0.0, 1.0, 0.0};
    l_joint_publisher_->publish(l_joint_msg_);
}
void XRNode::SendRResetArmGoal() {
    r_joint_msg_.header.stamp = this->now();
    r_joint_msg_.position = {0.0, 0.268, -0.75, 0.0, 1.0, 0.0};
    r_joint_publisher_->publish(r_joint_msg_);
}
void XRNode::SendExtendArmGoal() {
    Eigen::VectorXd q_solution(6);
    q_solution << 0.0, 0.268, -0.75, 0.0, 1.0, 0.0;

    rclcpp::Time stamp = this->now();
    l_joint_msg_.header.stamp = stamp;
    r_joint_msg_.header.stamp = stamp;

    // 关键修复：调整 position 向量大小
    l_joint_msg_.position.resize(6);
    r_joint_msg_.position.resize(6);

    for (int i = 0; i < 6; ++i) {
        l_joint_msg_.position[i] = q_solution(i);
        r_joint_msg_.position[i] = q_solution(i);
    }

    // 检查发布器是否有效
    if (l_joint_publisher_) {
        l_joint_publisher_->publish(l_joint_msg_);
    }
    if (r_joint_publisher_) {
        r_joint_publisher_->publish(r_joint_msg_);
    }
}
void XRNode::SendRetractArmGoal() {
    l_joint_msg_.header.stamp = this->now();
    l_joint_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.3, 0.0};
    l_joint_publisher_->publish(l_joint_msg_);
    r_joint_msg_.header.stamp = this->now();
    r_joint_msg_.position = {0.0, 0.0, 0.0, 0.0, 0.3, 0.0};
    r_joint_publisher_->publish(r_joint_msg_);
}
void XRNode::EnableArmServo(bool enable) {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enable;
    l_enable_agx_arm_client_->async_send_request(request);
    r_enable_agx_arm_client_->async_send_request(request);
}

void XRNode::publishGripperState(double gripper_value, bool is_left) {
    if (is_left) {
        l_gripper_msg_.header.stamp = this->now();
        l_gripper_msg_.position[0] = gripper_value;
        l_gripper_joint_publisher_->publish(l_gripper_msg_);
    } else {
        r_gripper_msg_.header.stamp = this->now();
        r_gripper_msg_.position[0] = gripper_value;
        r_gripper_joint_publisher_->publish(r_gripper_msg_);
    }
}

void XRNode::OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData) {
    (void)context;
    switch (type) {
    case PXREAServerConnect:
        std::cout << "server connect" << std::endl;
        break;
    case PXREAServerDisconnect:
        std::cout << "server disconnect" << std::endl;
        break;
    case PXREADeviceFind:
        std::cout << "device find" << (const char*)userData << std::endl;
        break;
    case PXREADeviceMissing:
        std::cout << "device missing" << (const char*)userData << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect" << (const char*)userData << status << std::endl;
        break;
    case PXREADeviceStateJson: {
        auto& dsj = *((PXREADevStateJson*)userData);
        try {
            auto json_obj = json::parse(dsj.stateJson);
            auto value_str = json_obj["value"].get<std::string>();
            auto value_obj = json::parse(value_str);

            auto custom_msg = xr_msgs::msg::Custom();
            custom_msg.timestamp_ns = value_obj["timeStampNs"].get<uint64_t>();
            custom_msg.input = value_obj["Input"].get<int>();

            // head
            auto head_msg = xr_msgs::msg::Head();
            if (value_obj.contains("Head")) {
                auto head_j = value_obj["Head"];
                std::vector<float> head_pose = stringToFloatVector(head_j["pose"].get<std::string>());
                if (head_pose.size() != 7) {
                    std::cerr << "Parse failed: head pose data length != 7" << std::endl;
                }
                std::copy(head_pose.begin(), head_pose.end(), head_msg.pose.data());
                head_msg.status = head_j["status"].get<int>();
            } else {
                head_msg.status = -1;
            }
            custom_msg.head = head_msg;

            // controller
            if (value_obj.contains("Controller")) {
                for (auto& element : value_obj["Controller"].items()) {
                    auto controller_msg = xr_msgs::msg::Controller();
                    auto ctrl_j = element.value();

                    controller_msg.axis_x = ctrl_j["axisX"].get<float>();
                    controller_msg.axis_y = ctrl_j["axisY"].get<float>();
                    controller_msg.axis_click = ctrl_j["axisClick"].get<bool>();
                    controller_msg.gripper = ctrl_j["grip"].get<float>();
                    controller_msg.trigger = ctrl_j["trigger"].get<float>();
                    controller_msg.primary_button = ctrl_j["primaryButton"].get<bool>();
                    controller_msg.secondary_button = ctrl_j["secondaryButton"].get<bool>();
                    controller_msg.menu_button = ctrl_j["menuButton"].get<bool>();
                    std::vector<float> ctrl_pose = stringToFloatVector(ctrl_j["pose"].get<std::string>());
                    if (ctrl_pose.size() != 7) {
                        std::cerr << "Parse failed: ctrl pose data length != 7" << std::endl;
                    }
                    std::copy(ctrl_pose.begin(), ctrl_pose.end(), controller_msg.pose.data());
                    controller_msg.status = 3;

                    if (element.key() == "left") {
                        custom_msg.left_controller = controller_msg;
                    } else {
                        custom_msg.right_controller = controller_msg;
                    }
                }
            } else {
                auto left_controller_msg = xr_msgs::msg::Controller();
                auto right_controller_msg = xr_msgs::msg::Controller();
                left_controller_msg.status = -1;
                right_controller_msg.status = -1;
                custom_msg.left_controller = left_controller_msg;
                custom_msg.right_controller = right_controller_msg;
            }

            publisher_->publish(custom_msg);

            if (custom_msg.left_controller.gripper == 1.0f) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.stamp = this->now();
                ps.header.frame_id = "Pico";

                ps.pose.position.x = custom_msg.left_controller.pose[0];
                ps.pose.position.y = custom_msg.left_controller.pose[1];
                ps.pose.position.z = custom_msg.left_controller.pose[2];
                ps.pose.orientation.x = custom_msg.left_controller.pose[3];
                ps.pose.orientation.y = custom_msg.left_controller.pose[4];
                ps.pose.orientation.z = custom_msg.left_controller.pose[5];
                ps.pose.orientation.w = custom_msg.left_controller.pose[6];

                if (l_ctl_init == false) {
                    l_real_has_new_pose_ = false;
                    l_real_has_new_joint_ = false;
                    l_ctl_init_pose.header.stamp = ps.header.stamp;
                    l_ctl_init_pose.header.frame_id = ps.header.frame_id;
                    l_ctl_init_pose.pose = ps.pose;
                    l_ctl_init = true;

                    RCLCPP_INFO(this->get_logger(), 
                        "\033[1;33m[LEFT CTRL INIT] Pose: [x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f]\033[0m",
                        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                        ps.pose.orientation.w);
                }
                else if (l_ctl_init == true && l_real_has_new_pose_ == true && l_real_has_new_joint_ == true) {

                    double dx_ctl = ps.pose.position.x - l_ctl_init_pose.pose.position.x;
                    double dy_ctl = ps.pose.position.y - l_ctl_init_pose.pose.position.y;
                    double dz_ctl = ps.pose.position.z - l_ctl_init_pose.pose.position.z;
                    
                    double dx_real = -dz_ctl;
                    double dy_real = -dx_ctl;
                    double dz_real = dy_ctl;
                    
                    tf2::Quaternion q_ctl_init(
                        l_ctl_init_pose.pose.orientation.x,
                        l_ctl_init_pose.pose.orientation.y,
                        l_ctl_init_pose.pose.orientation.z,
                        l_ctl_init_pose.pose.orientation.w);
                    
                    tf2::Quaternion q_ctl_current(
                        ps.pose.orientation.x,
                        ps.pose.orientation.y,
                        ps.pose.orientation.z,
                        ps.pose.orientation.w);
                    
                    // 计算手柄相对旋转（从初始到当前）
                    tf2::Quaternion q_ctl_rel = q_ctl_init.inverse() * q_ctl_current;
                    q_ctl_rel.normalize();
                    tf2::Matrix3x3 R_ctl_rel(q_ctl_rel);

                    double delRoll, delPitch, delYaw;
                    tf2::Matrix3x3(q_ctl_rel).getRPY(delRoll, delPitch, delYaw);
                    delRoll  *= 180.0 / M_PI;
                    delPitch *= 180.0 / M_PI;
                    delYaw   *= 180.0 / M_PI;

                    tf2::Matrix3x3 R_ctl_to_real(
                        0,  0, -1,   // 手柄Z后 → 机器人X后
                        -1, 0,  0,   // 手柄X右 → 机器人Y右
                        0,  1,  0    // 手柄Y上 → 机器人Z上
                    );
                    tf2::Matrix3x3 R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose();

                    tf2::Quaternion q_real_rel;
                    R_real_rel.getRotation(q_real_rel);
                    q_real_rel.normalize();

                    geometry_msgs::msg::PoseStamped result;
                    result.header.stamp = this->now();
                    result.header.frame_id = "real";


                    result.pose.position.x = l_real_pose.pose.position.x + dx_real;
                    result.pose.position.y = l_real_pose.pose.position.y + dy_real;
                    result.pose.position.z = l_real_pose.pose.position.z + dz_real;

                    // result.pose.position.x = l_real_pose.pose.position.x + dx_real;
                    // result.pose.position.y = l_real_pose.pose.position.y + dy_real;
                    // result.pose.position.z = l_real_pose.pose.position.z + dz_real;

                    tf2::Quaternion q_real_current(
                        l_real_pose.pose.orientation.x,
                        l_real_pose.pose.orientation.y,
                        l_real_pose.pose.orientation.z,
                        l_real_pose.pose.orientation.w);

                    double initRoll, initPitch, initYaw;
                    tf2::Matrix3x3(q_real_current).getRPY(initRoll, initPitch, initYaw);
                    initRoll  *= 180.0 / M_PI;
                    initPitch *= 180.0 / M_PI;
                    initYaw   *= 180.0 / M_PI;

                    // tf2::Quaternion q_real_new = q_real_current * q_real_rel;   // 相对旋转作用于机器人末端坐标系
                    tf2::Quaternion q_real_new = q_real_rel * q_real_current;    // 将手柄的相对旋转映射到机器人基坐标系
                    q_real_new.normalize();

                    result.pose.orientation.x = q_real_new.x();
                    result.pose.orientation.y = q_real_new.y();
                    result.pose.orientation.z = q_real_new.z();
                    result.pose.orientation.w = q_real_new.w();

                    // ========== IK 求解 ==========
                    Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
                    target_pose(0,3) = result.pose.position.x;
                    target_pose(1,3) = result.pose.position.y;
                    target_pose(2,3) = result.pose.position.z;
                    
                    tf2::Quaternion q_result(
                        result.pose.orientation.x,
                        result.pose.orientation.y,
                        result.pose.orientation.z,
                        result.pose.orientation.w);
                    tf2::Matrix3x3 rot_mat(q_result);
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            target_pose(i,j) = rot_mat[i][j];
                        }
                    }
                    
                    Eigen::VectorXd q_guess = last_q_left_;
                    Eigen::VectorXd q_solution;
                    bool ik_success = ik_solver_left_->solve(target_pose, q_guess, q_solution, last_q_left_);
                    
                    if (ik_success) {
                        if (isJointJump(q_solution, last_q_left_, true)) {
                            // 发生跳变，拒绝此次解
                            // RCLCPP_WARN(this->get_logger(), "\033[1;31mLeft arm IK solution rejected due to joint jump! Keeping previous state.\033[0m");
                        } else {
                            // 没有跳变，接受此解
                            last_q_left_ = q_solution;
                            is_first_valid_ik_left_ = false; // 标记已经有过有效解了


                            tf2::Quaternion q(
                                result.pose.orientation.x,
                                result.pose.orientation.y,
                                result.pose.orientation.z,
                                result.pose.orientation.w
                            );
                            double roll, pitch, yaw;
                            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            roll  *= 180.0 / M_PI;
                            pitch *= 180.0 / M_PI;
                            yaw   *= 180.0 / M_PI;


                            RCLCPP_INFO(this->get_logger(),
                                "\033[1;33m[LEFT CTRL DEBUG]\033[0m\n"
                                "┌──────────────────────────────────┬─────────────────────────────────────────────────────────┐\n"
                                "│ Real world ini pos               │ x:%8.6f  y:%8.6f  z:%8.6f                              │\n"
                                "│ delta pos                        │ x:%8.6f  y:%8.6f  z:%8.6f                              │\n"
                                "│ Real world new pos               │ x:%8.6f  y:%8.6f  z:%8.6f                              │\n"
                                "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                                "│ Real world ini rot               │ r:%8.2f  p:%8.2f  y:%8.2f                              │\n"
                                "│ del rot                          │ r:%8.2f  p:%8.2f  y:%8.2f                              │\n"
                                "│ Real world new rot               │ r:%8.2f  p:%8.2f  y:%8.2f                              │\n"
                                "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                                "│ Left arm topic                   │ %-49s │\n"
                                "│ LEFT arm joint                   │ %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f │\n"
                                "└──────────────────────────────────┴─────────────────────────────────────────────────────────┘",
                                l_real_pose.pose.position.x, l_real_pose.pose.position.y, l_real_pose.pose.position.z,
                                dx_real, dy_real, dz_real,
                                result.pose.position.x, result.pose.position.y, result.pose.position.z,

                                initRoll, initPitch, initYaw,
                                delRoll, delPitch, delYaw,
                                roll, pitch, yaw,
                                
                                l_joint_publisher_->get_topic_name(),
                                q_solution(0), q_solution(1), q_solution(2), q_solution(3), q_solution(4), q_solution(5)
                            );

                            publishJointState(q_solution, true);  // true表示左臂
                        }
                    } else {
                        // RCLCPP_WARN(this->get_logger(), "\033[1;31mLeft arm IK solve failed!\033[0m");
                    }
                }
            }
            else {
                if (l_ctl_init) {
                    l_ctl_init = false;
                    RCLCPP_INFO(this->get_logger(), "\033[1;31mLeft trigger released, calling emergency stop service\033[0m");
                    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                    l_emergency_stop_client_->async_send_request(request);
                }
            }

            if (custom_msg.left_controller.trigger == 1.0f && left_gripper == false) {
                double val = 0.0f;
                publishGripperState(val, true);
                left_gripper = true;
                RCLCPP_INFO(this->get_logger(), "%s Left gripper: [%f]", l_gripper_joint_publisher_->get_topic_name(), val);
            }
            else if (custom_msg.left_controller.trigger != 1.0f && left_gripper == true) {
                double val = gripper_open_angle_;
                publishGripperState(val, true);
                left_gripper = false;
                RCLCPP_INFO(this->get_logger(), "%s Left gripper: [%f]", l_gripper_joint_publisher_->get_topic_name(), val);
            }





            if (custom_msg.right_controller.gripper == 1.0f) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.stamp = this->now();
                ps.header.frame_id = "Pico";

                ps.pose.position.x = custom_msg.right_controller.pose[0];
                ps.pose.position.y = custom_msg.right_controller.pose[1];
                ps.pose.position.z = custom_msg.right_controller.pose[2];
                ps.pose.orientation.x = custom_msg.right_controller.pose[3];
                ps.pose.orientation.y = custom_msg.right_controller.pose[4];
                ps.pose.orientation.z = custom_msg.right_controller.pose[5];
                ps.pose.orientation.w = custom_msg.right_controller.pose[6];

                if (r_ctl_init == false) {
                    r_real_has_new_pose_ = false;
                    r_real_has_new_joint_ = false;
                    r_ctl_init_pose.header.stamp = ps.header.stamp;
                    r_ctl_init_pose.header.frame_id = ps.header.frame_id;
                    r_ctl_init_pose.pose = ps.pose;
                    r_ctl_init = true;
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "\033[1;34m[RIGHT CTRL INIT] Pose: [x:%f, y:%f, z:%f, qx:%f, qy:%f, qz:%f, qw:%f]\033[0m",
                        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z,
                        ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z,
                        ps.pose.orientation.w);
                }
                else if (r_ctl_init == true && r_real_has_new_pose_ == true && r_real_has_new_joint_ == true) {
                    double dx_ctl = ps.pose.position.x - r_ctl_init_pose.pose.position.x;
                    double dy_ctl = ps.pose.position.y - r_ctl_init_pose.pose.position.y;
                    double dz_ctl = ps.pose.position.z - r_ctl_init_pose.pose.position.z;
                    
                    double dx_real = -dz_ctl;
                    double dy_real = -dx_ctl;
                    double dz_real = dy_ctl;
                    
                    tf2::Quaternion q_ctl_init(
                        r_ctl_init_pose.pose.orientation.x,
                        r_ctl_init_pose.pose.orientation.y,
                        r_ctl_init_pose.pose.orientation.z,
                        r_ctl_init_pose.pose.orientation.w);
                    
                    tf2::Quaternion q_ctl_current(
                        ps.pose.orientation.x,
                        ps.pose.orientation.y,
                        ps.pose.orientation.z,
                        ps.pose.orientation.w);
                    
                    // 计算手柄相对旋转（从初始到当前）
                    tf2::Quaternion q_ctl_rel = q_ctl_init.inverse() * q_ctl_current;
                    q_ctl_rel.normalize();
                    tf2::Matrix3x3 R_ctl_rel(q_ctl_rel);

                    double delRoll, delPitch, delYaw;
                    tf2::Matrix3x3(q_ctl_rel).getRPY(delRoll, delPitch, delYaw);
                    delRoll  *= 180.0 / M_PI;
                    delPitch *= 180.0 / M_PI;
                    delYaw   *= 180.0 / M_PI;

                    tf2::Matrix3x3 R_ctl_to_real(
                        0,  0, -1,   // 手柄Z后 → 机器人X后
                        -1, 0,  0,   // 手柄X右 → 机器人Y右
                        0,  1,  0    // 手柄Y上 → 机器人Z上
                    );
                    tf2::Matrix3x3 R_real_rel = R_ctl_to_real * R_ctl_rel * R_ctl_to_real.transpose();

                    tf2::Quaternion q_real_rel;
                    R_real_rel.getRotation(q_real_rel);
                    q_real_rel.normalize();

                    geometry_msgs::msg::PoseStamped result;
                    result.header.stamp = this->now();
                    result.header.frame_id = "real";


                    result.pose.position.x = r_real_pose.pose.position.x + dx_real;
                    result.pose.position.y = r_real_pose.pose.position.y + dy_real;
                    result.pose.position.z = r_real_pose.pose.position.z + dz_real;

                    // result.pose.position.x = r_real_pose.pose.position.x + dx_real;
                    // result.pose.position.y = r_real_pose.pose.position.y + dy_real;
                    // result.pose.position.z = r_real_pose.pose.position.z + dz_real;

                    tf2::Quaternion q_real_current(
                        r_real_pose.pose.orientation.x,
                        r_real_pose.pose.orientation.y,
                        r_real_pose.pose.orientation.z,
                        r_real_pose.pose.orientation.w);

                    double initRoll, initPitch, initYaw;
                    tf2::Matrix3x3(q_real_current).getRPY(initRoll, initPitch, initYaw);
                    initRoll  *= 180.0 / M_PI;
                    initPitch *= 180.0 / M_PI;
                    initYaw   *= 180.0 / M_PI;

                    tf2::Quaternion q_real_new = q_real_rel * q_real_current;    // 将手柄的相对旋转映射到机器人基坐标系
                    q_real_new.normalize();

                    result.pose.orientation.x = q_real_new.x();
                    result.pose.orientation.y = q_real_new.y();
                    result.pose.orientation.z = q_real_new.z();
                    result.pose.orientation.w = q_real_new.w();

                    // ========== 右臂 IK 求解 ==========
                    Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
                    target_pose(0,3) = result.pose.position.x;
                    target_pose(1,3) = result.pose.position.y;
                    target_pose(2,3) = result.pose.position.z;
                    
                    tf2::Quaternion q_result(
                        result.pose.orientation.x,
                        result.pose.orientation.y,
                        result.pose.orientation.z,
                        result.pose.orientation.w);
                    tf2::Matrix3x3 rot_mat(q_result);
                    for (int i = 0; i < 3; ++i) {
                        for (int j = 0; j < 3; ++j) {
                            target_pose(i,j) = rot_mat[i][j];
                        }
                    }
                    
                    Eigen::VectorXd q_guess = last_q_right_;
                    Eigen::VectorXd q_solution;
                    bool ik_success = ik_solver_right_->solve(target_pose, q_guess, q_solution, last_q_right_);
                    
                    if (ik_success) {
                        if (isJointJump(q_solution, last_q_right_, false)) {
                            // RCLCPP_WARN(this->get_logger(), 
                            //     "\033[1;31mRight arm IK solution rejected due to joint jump! Keeping previous state.\033[0m");
                        } else {
                            last_q_right_ = q_solution;
                            is_first_valid_ik_right_ = false;


                            tf2::Quaternion q(
                                result.pose.orientation.x,
                                result.pose.orientation.y,
                                result.pose.orientation.z,
                                result.pose.orientation.w
                            );
                            double roll, pitch, yaw;
                            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                            roll  *= 180.0 / M_PI;
                            pitch *= 180.0 / M_PI;
                            yaw   *= 180.0 / M_PI;


                            RCLCPP_INFO(this->get_logger(),
                                "\033[1;33m[RIGHT CTRL DEBUG]\033[0m\n"
                                "┌──────────────────────────────────┬─────────────────────────────────────────────────────────┐\n"
                                "│ Real world ini pos               │ x:%8.6f  y:%8.6f  z:%8.6f                              │\n"
                                "│ delta pos                        │ x:%8.6f  y:%8.6f  z:%8.6f                              │\n"
                                "│ Real world new pos               │ x:%8.6f  y:%8.6f  z:%8.6f                              │\n"
                                "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                                "│ Real world ini rot               │ r:%8.2f  p:%8.2f  y:%8.2f                              │\n"
                                "│ del rot                          │ r:%8.2f  p:%8.2f  y:%8.2f                              │\n"
                                "│ Real world new rot               │ r:%8.2f  p:%8.2f  y:%8.2f                              │\n"
                                "├──────────────────────────────────┼─────────────────────────────────────────────────────────┤\n"
                                "│ Right arm topic                  │ %-49s │\n"
                                "│ Right arm joint                  │ %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f │\n"
                                "└──────────────────────────────────┴─────────────────────────────────────────────────────────┘",
                                r_real_pose.pose.position.x, r_real_pose.pose.position.y, r_real_pose.pose.position.z,
                                dx_real, dy_real, dz_real,
                                result.pose.position.x, result.pose.position.y, result.pose.position.z,

                                initRoll, initPitch, initYaw,
                                delRoll, delPitch, delYaw,
                                roll, pitch, yaw,

                                r_joint_publisher_->get_topic_name(),
                                q_solution(0), q_solution(1), q_solution(2), q_solution(3), q_solution(4), q_solution(5)
                            );

                            publishJointState(q_solution, false);  // false表示右臂
                        }
                    } else {
                        RCLCPP_WARN(this->get_logger(), "\033[1;31mRight arm IK solve failed!\033[0m");
                    }
                }
            } else {
                // 右臂释放逻辑
                if (r_ctl_init) {
                    r_ctl_init = false;
                    RCLCPP_INFO(this->get_logger(), "\033[1;31mRight trigger released, calling emergency stop service\033[0m");
                    auto request = std::make_shared<std_srvs::srv::Empty::Request>();
                    r_emergency_stop_client_->async_send_request(request);
                }
            }

            if (custom_msg.right_controller.trigger == 1.0f && right_gripper == false) {
                double val = 0.0f;
                publishGripperState(val, false);
                right_gripper = true;
                RCLCPP_INFO(this->get_logger(), "%s Right gripper: [%f]", r_gripper_joint_publisher_->get_topic_name(), val);
            }
            else if (custom_msg.right_controller.trigger != 1.0f && right_gripper == true) {
                double val = gripper_open_angle_;
                publishGripperState(val, false);
                right_gripper = false;
                RCLCPP_INFO(this->get_logger(), "%s Right gripper: [%f]", r_gripper_joint_publisher_->get_topic_name(), val);
            }




            // ==== 长按检测逻辑 ====
            auto current_time = std::chrono::steady_clock::now();

            // 长按 X & A 双臂伸臂
            if (custom_msg.left_controller.primary_button && custom_msg.right_controller.primary_button) {
                if (!dual_primary_pressed_) {
                    dual_primary_pressed_ = true;
                    dual_primary_press_start_ = current_time;
                    dual_primary_press_sent_ = false;  // 重置发送标志
                    // RCLCPP_INFO(this->get_logger(), "Dual primary button pressed, starting timer...");
                } else {
                    // 按钮持续按住，检查是否超过pressed_time秒
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - dual_primary_press_start_).count();

                    if (duration >= pressed_time && !dual_primary_press_sent_) {
                        RCLCPP_INFO(this->get_logger(), 
                            "Dual primary button held for %ld seconds, sending goal", 
                            duration);
                        dual_primary_press_sent_ = true;  // 标记已发送，防止重复发送
                        SendExtendArmGoal();
                    }
                }
            } else {
                // 按钮未被按下，重置状态
                if (dual_primary_pressed_) {
                    // RCLCPP_INFO(this->get_logger(), "Dual secondary button released");
                }
                dual_primary_pressed_ = false;
                dual_primary_press_sent_ = false;
            }

            // 左复位
            if (custom_msg.left_controller.primary_button && !custom_msg.right_controller.primary_button) {
                if (!left_primary_pressed_) {
                    left_primary_pressed_ = true;
                    left_primary_press_start_ = current_time;
                    left_primary_press_sent_ = false;  // 重置发送标志
                    // RCLCPP_INFO(this->get_logger(), "Left primary button pressed, starting timer...");
                } else {
                    // 按钮持续按住，检查是否超过pressed_time秒
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - left_primary_press_start_).count();
                    
                    if (duration >= pressed_time && !left_primary_press_sent_) {
                        // 长按超过pressed_time秒且未发送过，执行伸臂任务
                        RCLCPP_INFO(this->get_logger(), 
                            "Left primary button held for %ld seconds, sending goal", 
                            duration);
                        left_primary_press_sent_ = true;  // 标记已发送，防止重复发送
                        SendLResetArmGoal();
                    }
                }
            } else {
                // 按钮未被按下，重置状态
                if (left_primary_pressed_) {
                    // RCLCPP_INFO(this->get_logger(), "Left primary button released");
                }
                left_primary_pressed_ = false;
                left_primary_press_sent_ = false;
            }

            // 右复位
            if (custom_msg.right_controller.primary_button && !custom_msg.left_controller.primary_button) {
                if (!right_primary_pressed_) {
                    right_primary_pressed_ = true;
                    right_primary_press_start_ = current_time;
                    right_primary_press_sent_ = false;  // 重置发送标志
                    // RCLCPP_INFO(this->get_logger(), "Right primary button pressed, starting timer...");
                } else {
                    // 按钮持续按住，检查是否超过pressed_time秒
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - right_primary_press_start_).count();
                    
                    if (duration >= pressed_time && !right_primary_press_sent_) {
                        // 长按超过pressed_time秒且未发送过，执行收臂任务
                        RCLCPP_INFO(this->get_logger(), 
                            "Right primary button held for %ld seconds, sending goal", 
                            duration);
                        right_primary_press_sent_ = true;  // 标记已发送，防止重复发送
                        SendRResetArmGoal();
                    }
                }
            } else {
                // 按钮未被按下，重置状态
                if (right_primary_pressed_) {
                    // RCLCPP_INFO(this->get_logger(), "Right primary button released");
                }
                right_primary_pressed_ = false;
                right_primary_press_sent_ = false;
            }


            // 长按 Y & B 使能
            if (custom_msg.left_controller.secondary_button && custom_msg.right_controller.secondary_button) {
                if (!dual_secondary_pressed_) {
                    dual_secondary_pressed_ = true;
                    dual_secondary_press_start_ = current_time;
                    dual_secondary_press_sent_ = false;  // 重置发送标志
                    // RCLCPP_INFO(this->get_logger(), "Dual secondary button pressed, starting timer...");
                } else {
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - dual_secondary_press_start_).count();

                    if (duration >= pressed_time && !dual_secondary_press_sent_) {
                        RCLCPP_INFO(this->get_logger(), 
                            "Dual secondary button held for %ld seconds, sending goal", 
                            duration);
                        dual_secondary_press_sent_ = true;  // 标记已发送，防止重复发送
                        EnableArmServo(true);
                    }
                }
            } else {
                // 按钮未被按下，重置状态
                if (dual_secondary_pressed_) {
                    // RCLCPP_INFO(this->get_logger(), "Dual secondary button released");
                }
                dual_secondary_pressed_ = false;
                dual_secondary_press_sent_ = false;
            }

            // Y 双臂收臂
            if (custom_msg.left_controller.secondary_button && !custom_msg.right_controller.secondary_button) {
                if (!left_secondary_pressed_) {
                    left_secondary_pressed_ = true;
                    left_secondary_press_start_ = current_time;
                    left_secondary_press_sent_ = false;  // 重置发送标志
                    // RCLCPP_INFO(this->get_logger(), "Left secondary button pressed, starting timer...");
                } else {
                    // 按钮持续按住，检查是否超过pressed_time秒
                    auto duration = std::chrono::duration_cast<std::chrono::seconds>(
                        current_time - left_secondary_press_start_).count();
                    
                    if (duration >= pressed_time && !left_secondary_press_sent_) {
                        // 长按超过pressed_time秒且未发送过，执行伸臂任务
                        RCLCPP_INFO(this->get_logger(), 
                            "Left secondary button held for %ld seconds, sending goal", 
                            duration);
                        left_secondary_press_sent_ = true;  // 标记已发送，防止重复发送
                        SendRetractArmGoal();
                    }
                }
            } else {
                // 按钮未被按下，重置状态
                if (left_secondary_pressed_) {
                    // RCLCPP_INFO(this->get_logger(), "Left secondary button released");
                }
                left_secondary_pressed_ = false;
                left_secondary_press_sent_ = false;
            }

            // // B 开启力控
            // if (custom_msg.right_controller.secondary_button && !custom_msg.left_controller.secondary_button) {
            //     if (!right_secondary_pressed_) {
            //         right_secondary_pressed_ = true;
            //         right_secondary_press_start_ = current_time;
            //         right_secondary_press_sent_ = false;  // 重置发送标志
            //         // RCLCPP_INFO(this->get_logger(), "Right secondary button pressed, starting timer...");
            //     } else {
            //         // 按钮持续按住，检查是否超过pressed_time秒
            //         auto duration = std::chrono::duration_cast<std::chrono::seconds>(
            //             current_time - right_secondary_press_start_).count();
                    
            //         if (duration >= pressed_time && !right_secondary_press_sent_) {
            //             // 长按超过pressed_time秒且未发送过，执行收臂任务
            //             RCLCPP_INFO(this->get_logger(), 
            //                 "Right secondary button held for %ld seconds, sending goal", 
            //                 duration);
            //             right_secondary_press_sent_ = true;  // 标记已发送，防止重复发送
            //             EnableArmCompliance(true);
            //         }
            //     }
            // } else {
            //     // 按钮未被按下，重置状态
            //     if (right_secondary_pressed_) {
            //         // RCLCPP_INFO(this->get_logger(), "Right secondary button released");
            //     }
            //     right_secondary_pressed_ = false;
            //     right_secondary_press_sent_ = false;
            // }


        } catch (const std::exception& e) {
            std::cerr << "Parse failed: " << e.what() << std::endl;
        }
        break;
    }
    case PXREADeviceCustomMessage:
        std::cout << "device custom message" << std::endl;
        break;
    case PXREAFullMask:
        std::cout << "full mask" << std::endl;
        break;
    }
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto xrNode = std::make_shared<XRNode>();
    g_callback = [&xrNode] (void* context, PXREAClientCallbackType type, int status, void* userData) {
        xrNode->OnPXREAClientCallback(context, type, status, userData);
    };
    PXREAInit(NULL, callbackForwarder, PXREAFullMask);
    rclcpp::spin(xrNode);
    PXREADeinit();

    rclcpp::shutdown();
    return 0;
}
