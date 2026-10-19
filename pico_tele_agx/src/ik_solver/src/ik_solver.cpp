#include <pinocchio/autodiff/casadi.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/log.hpp>
#include <casadi/casadi.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <iostream>
#include <vector>

std::vector<double> DMToStdVector(const casadi::DM& dm)
{
    std::vector<double> v(dm.size1());
    for (int i = 0; i < dm.size1(); ++i)
        v[i] = static_cast<double>(dm(i));
    return v;
}

casadi::DM matrix_to_dm(const Eigen::Matrix4d& matrix)
{
    casadi::DM ret = casadi::DM::eye(4);
    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            ret(i,j) = matrix(i,j);
        }
    }
    return ret;
}

int main()
{
    using Scalar   = double;
    using ADScalar = casadi::SX;

    using Model   = pinocchio::ModelTpl<Scalar>;
    using Data    = pinocchio::DataTpl<Scalar>;

    using ADModel = pinocchio::ModelTpl<ADScalar>;
    using ADData  = pinocchio::DataTpl<ADScalar>;

    // =========================================================
    // 1. 构建 Pinocchio 模型（URDF）- 单臂七轴
    // =========================================================
    Model model;
    std::string urdf_path = "/projects/pico_tele_agx/src/agx_arm_ros/src/agx_arm_description/agx_arm_urdf/nero/urdf/nero_description.urdf";  
    pinocchio::urdf::buildModel(urdf_path, model);
    std::cout << "Model loaded. nq = " << model.nq
                << ", nv = " << model.nv << std::endl;
    
    // 选择末端 frame（单臂七轴，末端为joint7）
    std::string ee_frame_name = "joint7"; 
    pinocchio::FrameIndex ee_id = model.getFrameId(ee_frame_name);
    std::cout << "End-effector frame: " << ee_frame_name << " (id: " << ee_id << ")" << std::endl;
    
    // =========================================================
    // 2. Cast 为 CasADi 符号模型
    // =========================================================
    ADModel ad_model = model.cast<ADScalar>();
    ADData  ad_data(ad_model);

    // =========================================================
    // 3. 定义 CasADi 符号关节变量 q
    // =========================================================
    casadi::SX q_sym = casadi::SX::sym("q", model.nq);
    casadi::SX Tf_sym = casadi::SX::sym("tf", 4, 4);  // 单臂目标位姿
    
    ADModel::ConfigVectorType q_ad(model.nq);
    pinocchio::casadi::copy(q_sym, q_ad);
    
    // =========================================================
    // 4. 符号前向运动学
    // =========================================================
    pinocchio::forwardKinematics(ad_model, ad_data, q_ad);
    pinocchio::updateFramePlacements(ad_model, ad_data);

    // 获取末端位姿（单臂）
    pinocchio::FrameIndex hand_id = ad_model.getFrameId(ee_frame_name);
    
    // 构建平移误差函数
    auto tcol = [](const casadi::SX& T4x4){
        return T4x4(casadi::Slice(0,3), casadi::Slice(3,4));  // 3x1
    };

    auto eig3_to_sx = [](const Eigen::Matrix<casadi::SX,3,1>& v){
        casadi::SX out(3,1);
        for(int i=0;i<3;++i) out(i) = v(i);
        return out;
    };

    // 平移误差
    casadi::SX p = eig3_to_sx(ad_data.oMf[hand_id].translation());
    casadi::SX p_err = p - tcol(Tf_sym);
    casadi::SX translational_error_expr = p_err;
    
    casadi::Function translational_error(
        "translational_error",
        {q_sym, Tf_sym},
        {translational_error_expr}
    );

    // ===========================================================
    // 构建旋转误差（SO(3) 对数映射形式）
    // ===========================================================
    const Eigen::Matrix<casadi::SX,3,3>& R = ad_data.oMf[hand_id].rotation();
    
    // 期望旋转矩阵
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
    
    casadi::SX rot_err_sx = casadi::SX::vertcat({
        rot_err(0), rot_err(1), rot_err(2)
    });
    
    casadi::Function rotational_error(
        "rotational_error",
        {q_sym, Tf_sym},
        {rot_err_sx}
    );

    // =========================================================
    // 定义优化问题（单臂）
    // =========================================================
    casadi::Opti opti;
    casadi::MX var_q = opti.variable(model.nq);
    casadi::MX var_q_last = opti.parameter(model.nq);      // 上一时刻关节角
    casadi::MX param_tf = opti.parameter(4, 4);           // 目标位姿
    casadi::MX q_ref = opti.parameter(model.nq);          // 参考关节角（用于特定关节）
    casadi::MX joint_w = opti.parameter(model.nq);        // 关节权重
    
    // 平移误差 cost
    casadi::MXVector out1 = translational_error(casadi::MXVector{var_q, param_tf});
    casadi::MX translational_cost = casadi::MX::sumsqr(out1.at(0));
    
    // 旋转误差 cost
    casadi::MXVector out2 = rotational_error(casadi::MXVector{var_q, param_tf});
    casadi::MX rotational_cost = casadi::MX::sumsqr(out2.at(0));
    
    // 正则化 cost（关节角尽量小）
    casadi::MX regularization_cost = casadi::MX::sumsqr(var_q);
    
    // 平滑 cost（与上一时刻解接近）
    casadi::MX smooth_cost = casadi::MX::sumsqr(var_q - var_q_last);
    
    // 特定关节目标 / 加权跟踪
    casadi::MX joint_i_cost = casadi::MX::sumsqr(casadi::MX::diag(joint_w) * (var_q - q_ref));

    // 约束：关节限位
    Eigen::VectorXd q_min = model.lowerPositionLimit;
    Eigen::VectorXd q_max = model.upperPositionLimit;
    casadi::DM q_min_dm(q_min.size(), 1);
    casadi::DM q_max_dm(q_max.size(), 1);
    for (int i = 0; i < q_min.size(); ++i) {
        q_min_dm(i) = q_min(i);
        q_max_dm(i) = q_max(i);
    }
    opti.subject_to(opti.bounded(q_min_dm, var_q, q_max_dm));

    // 最小化代价函数（调整权重）
    opti.minimize(
        50.0 * translational_cost 
        + rotational_cost    // 旋转误差权重
        + 0.04 * smooth_cost
        + 0.06 * joint_i_cost
        // + 0.01 * regularization_cost  // 可选：正则化
    );
    
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
    opts["ipopt.max_wall_time"] = 1e-2; // 限时10ms完成求解
    opts["ipopt.warm_start_init_point"] = "yes";
    opts["ipopt.derivative_test"] = "none";
    opts["ipopt.jacobian_approximation"] = "exact";
    
    opti.solver("ipopt", opts);

    // 设置参考关节角和权重（根据实际机器人调整）
    std::vector<double> ref_j(model.nq, 0.0);  // 零位作为参考
    casadi::DM ref_data = casadi::DM::zeros(model.nq, 1);
    for (int i = 0; i < model.nq; ++i) ref_data(i) = ref_j[i];
    opti.set_value(q_ref, ref_data);

    std::vector<double> w_j(model.nq, 1.0);  // 所有权重为1
    casadi::DM w_data = casadi::DM::zeros(model.nq, 1);
    for (int i = 0; i < model.nq; ++i) w_data(i) = w_j[i];
    opti.set_value(joint_w, w_data);

    // =========================================================
    // 测试：给定目标位姿求逆解
    // =========================================================
    
    // 初始关节角猜测（可根据实际情况调整）
    std::vector<double> start_j(model.nq, 0.0);
    // 或者设置一些合理的初始值，例如：
    // start_j = {0.0, -0.5, 0.0, -1.0, 0.0, 0.5, 0.0};  // 示例值，需根据实际机器人调整
    
    casadi::DM init_data = casadi::DM::zeros(model.nq, 1);
    for (int i = 0; i < model.nq; ++i) init_data(i) = start_j[i];

    // 目标位姿（示例：单位矩阵，表示原点处无旋转）
    Eigen::Matrix4d target_pose = Eigen::Matrix4d::Identity();
    // 设置一个具体的目标位姿，例如：
    target_pose << 1, 0, 0, 0.5,   // x平移0.5m
                  0, 1, 0, 0.0,   // y平移0.0m
                  0, 0, 1, 0.3,   // z平移0.3m
                  0, 0, 0, 1;
    
    casadi::DM Tf_dm = matrix_to_dm(target_pose);

    // 设置优化问题参数
    opti.set_initial(var_q, init_data);
    opti.set_value(param_tf, Tf_dm);
    opti.set_value(var_q_last, init_data);

    // 求解
    std::cout << "\nSolving inverse kinematics..." << std::endl;
    auto t_start = std::chrono::high_resolution_clock::now();
    
    try {
        casadi::OptiSol sol = opti.solve();
        auto t_end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start);
        
        std::cout << "Solve time: " << duration.count() << " us" << std::endl;
        std::cout << "IPOPT iterations: " << sol.stats()["iter_count"] << std::endl;
        
        casadi::DM q_sol = sol.value(var_q);
        auto q_vec = DMToStdVector(q_sol);
        
        std::cout << "\nSolution joint angles (rad):" << std::endl;
        for (size_t i = 0; i < q_vec.size(); ++i) {
            std::cout << "q" << i << ": " << q_vec[i] << std::endl;
        }
        
        // // 计算误差
        // casadi::DM terr = translational_error(casadi::DMVector{q_sol, Tf_dm})[0];
        // casadi::DM rerr = rotational_error(casadi::DMVector{q_sol, Tf_dm})[0];
        
        // std::cout << "\nTranslational error (x,y,z): " << terr << std::endl;
        // std::cout << "Rotational error (rx,ry,rz): " << rerr << std::endl;
        
        // // 验证正运动学
        // Model model_verify;
        // pinocchio::urdf::buildModel(urdf_path, model_verify);
        // Data data_verify(model_verify);
        // Eigen::VectorXd q_verify(model_verify.nq);
        // for (int i = 0; i < model_verify.nq; ++i) q_verify(i) = q_vec[i];
        
        // pinocchio::forwardKinematics(model_verify, data_verify, q_verify);
        // pinocchio::updateFramePlacements(model_verify, data_verify);
        
        // std::cout << "\nVerified end-effector pose:" << std::endl;
        // std::cout << data_verify.oMf[ee_id].translation().transpose() << std::endl;
        // std::cout << data_verify.oMf[ee_id].rotation() << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}