#include "mpc_acc.h"
#include "math.h"
#include "iostream"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>
#include <chrono>

#define PI 3.1415926

using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 4, 1> Vector4d;

MatrixXd MPC_ACC::solve(
        Eigen::Vector4d X_k,
        std::vector<Eigen::Vector4d> X_r,
        std::vector<Eigen::Vector2d> U_r, const int N) {
    auto start = std::chrono::high_resolution_clock::now();
    
    // 状态向量现在是 [x, y, theta, v] (4维)
    // 控制向量是 [a, w] (2维) - 线加速度和角速度(无角加速度)
    
    vector<MatrixXd> A_r(N), B_r(N), A_multiply1(N);
    MatrixXd O_r(4 * N, 1);
    MatrixXd A_bar(4 * N, 4);
    MatrixXd X_ref(4 * N, 1);
    MatrixXd A_multiply2;
    MatrixXd B_bar = MatrixXd::Zero(4 * N, 2 * N);
    MatrixXd C_bar = MatrixXd::Identity(4 * N, 4 * N);
    MatrixXd A_r_init = MatrixXd::Zero(4, 4);
    MatrixXd B_r_init = MatrixXd::Zero(4, 2);
    MatrixXd eye_4 = MatrixXd::Identity(4, 4);
    
    // 权重矩阵
    MatrixXd Q = MatrixXd::Identity(4 * N, 4 * N);
    
    for (int i = 0; i < N; i++) {
        Q(4*i, 4*i) = omega0_;      // x
        Q(4*i+1, 4*i+1) = omega0_;  // y
        Q(4*i+2, 4*i+2) = omega0_/10.f;  // theta
        
        // 速度权重：根据配置决定是否加大终点权重
        if (i == N-1) {
            Q(4*i+3, 4*i+3) = omega0_;  // 终点速度权重增大，促使v_N -> 0
        } else {
            Q(4*i+3, 4*i+3) = omega0_;   // 中间步速度权重较小
        }
    }
    
    MatrixXd R = MatrixXd::Identity(2 * N, 2 * N);
    // 对R矩阵进行间隔赋值 - 加速度控制权重
    for (int i = 0; i < 2 * N; i++) {
        if (i % 2 == 0) {
            // 线加速度权重：终点处加大权重以实现终点加速度为0
            if (i == 2*(N-1)) {  // 最后一步的加速度
                R(i, i) = omega1_v_;  // 终点加速度权重增大
            } else {
                R(i, i) = omega1_v_;  // 线加速度权重
            }
        } else {
            R(i, i) = omega1_w_;  // 角速度权重
        }
    }

    for (int k = 0; k < N; k++) {
        A_r[k] = A_r_init;
        B_r[k] = B_r_init;
        
        // 状态转移矩阵 A - 4x4 矩阵
        // dx/dt = v*cos(theta)
        // dy/dt = v*sin(theta) 
        // dtheta/dt = w
        // dv/dt = a
        
        double theta_ref = X_r[k](2);
        double v_ref = X_r[k](3);  // 现在从X_r中获取参考速度
        double w_ref = U_r[k](1);
        
        // 位置对角度的偏导
        A_r[k](0, 2) = -v_ref * sin(theta_ref);  // dx/dtheta
        A_r[k](1, 2) = v_ref * cos(theta_ref);   // dy/dtheta
        
        // 位置对速度的偏导
        A_r[k](0, 3) = cos(theta_ref);  // dx/dv
        A_r[k](1, 3) = sin(theta_ref);  // dy/dv
        
        // 角度对时间的偏导（角速度来自控制输入）
        // dtheta/dt = w (w来自控制输入，不是状态变量)
        
        Vector4d temp_vec = -t_step_acc_ * A_r[k] * Vector4d(X_r[k](0), X_r[k](1), X_r[k](2), X_r[k](3));
        O_r.block<4, 1>(k * 4, 0) = temp_vec;
        
        A_r[k] = eye_4 + t_step_acc_ * A_r[k];
        
        // 控制输入矩阵 B - 4x2 矩阵
        // 控制输入是 [a, w]
        B_r[k](2, 1) = t_step_acc_;  // dtheta/dw = dt
        B_r[k](3, 0) = t_step_acc_;  // dv/da = dt
        
        // 参考状态 [x, y, theta, v]
        Vector4d x_ref_extended;
        x_ref_extended << X_r[k](0), X_r[k](1), X_r[k](2), X_r[k](3);
        X_ref.block<4, 1>(k * 4, 0) = x_ref_extended;

        if (k == 0) A_multiply1[k] = A_r[k];
        else A_multiply1[k] = A_multiply1[k - 1] * A_r[k];
        A_bar.block<4, 4>(4 * k, 0) = A_multiply1[k];
    }

    for (int k = 0; k < N; k++) {
        B_bar.block<4, 2>(4 * k, 2 * k) = B_r[k];
        A_multiply2 = eye_4;
        for (int i = 0; i < k; i++) {
            A_multiply2 = A_multiply2 * A_r[k - i];
            C_bar.block<4, 4>(4 * k, 4 * (k - 1 - i)) = A_multiply2;
            B_bar.block<4, 2>(4 * k, 2 * (k - 1 - i)) = A_multiply2 * B_r[k - 1 - i];
        }
    }

    // 当前状态已经是4维 [x, y, theta, v]
    MatrixXd E = A_bar * X_k + C_bar * O_r - X_ref;
    MatrixXd Hesse = 2 * (B_bar.transpose() * Q * B_bar + R);
    VectorXd gradient = 2 * B_bar.transpose() * Q * E;

    std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "MPC_ACC Problem formulation time taken: " << elapsed.count() << " ms" << std::endl;

    start = std::chrono::high_resolution_clock::now();
    
    // 使用OSQP求解器
    OsqpEigen::Solver solver;
    
    // 设置求解器参数
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    
    Eigen::SparseMatrix<double> sparse_H(Hesse.sparseView());
    
    // 构建约束矩阵：包括控制量约束和速度状态约束
    // 控制量约束：u = [a, w] 
    // 速度状态约束：v_min <= v_k <= v_max，其中 v_k = v_current + sum(a_i*dt)
    
    double current_v = X_k(3);  // 当前速度
    
    // 总约束数量：2*N（控制约束）+ N（速度约束，双边界）= 3*N
    int total_constraints = 3 * N;
    Eigen::SparseMatrix<double> sparse_A(total_constraints, 2 * N);
    
    // 设置控制量约束（前2*N行）
    for (int i = 0; i < 2 * N; i++) {
        sparse_A.insert(i, i) = 1.0;
    }
    
    // 设置速度约束（第2*N到3*N-1行）- 每个时刻一个双边约束
    for (int k = 0; k < N; k++) {
        // 速度约束：v_min <= v_k <= v_max
        // v_k = v_current + sum(a_i*dt) for i = 0 to k
        for (int i = 0; i <= k; i++) {
            sparse_A.insert(2*N + k, 2*i) = t_step_acc_;  // 累积加速度影响
        }
    }
    
    VectorXd lower_bound(total_constraints);
    VectorXd upper_bound(total_constraints);
    
    // 控制量约束
    for (int i = 0; i < 2 * N; i++) {
        if (i % 2 == 0) {
            // 线加速度约束
            lower_bound(i) = a_min_;
            upper_bound(i) = a_max_;
        } else {
            // 角速度约束
            lower_bound(i) = -w_max_;
            upper_bound(i) = w_max_;
        }
    }
    
    // 速度约束（第2*N到3*N-1行）- 双边界约束
    for (int k = 0; k < N; k++) {
        // 对于速度约束：v_min <= v_k <= v_max
        // v_k = v_current + sum(a_i*dt)
        // 所以：v_min <= v_current + sum(a_i*dt) <= v_max
        // 即：v_min - v_current <= sum(a_i*dt) <= v_max - v_current
        lower_bound(2*N + k) = v_min_ - current_v;  // 下界
        upper_bound(2*N + k) = v_max_ - current_v;  // 上界
    }
    
    // 设置问题数据
    solver.data()->setNumberOfVariables(2*N);
    solver.data()->setNumberOfConstraints(total_constraints);  // 使用新的约束数量
    if (!solver.data()->setHessianMatrix(sparse_H)) 
        std::cout << "MPC_ACC Problem failed to setHessianMatrix !" << std::endl;
    if (!solver.data()->setGradient(gradient)) 
        std::cout << "MPC_ACC Problem failed to setGradient !" << std::endl;
    if (!solver.data()->setLinearConstraintsMatrix(sparse_A)) 
        std::cout << "MPC_ACC Problem failed to setLinearConstraintsMatrix !" << std::endl;
    if (!solver.data()->setLowerBound(lower_bound)) 
        std::cout << "MPC_ACC Problem failed to setLowerBound !" << std::endl;
    if (!solver.data()->setUpperBound(upper_bound)) 
        std::cout << "MPC_ACC Problem failed to setUpperBound !" << std::endl;
    
    // 初始化求解器
    if (!solver.initSolver()) 
        cout << "MPC_ACC Problem failed to initSolver !" << std::endl;
    
    Eigen::VectorXd solution;
    // 执行求解
    if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        solution = solver.getSolution();
    } else {
        std::cout << "MPC_ACC Problem failed to solve!" << std::endl;
        // 返回零控制输入
        solution = VectorXd::Zero(2 * N);
    }
    
    elapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "MPC_ACC OSQP Time taken: " << elapsed.count() << " ms" << std::endl;
    
    // 构建结果矩阵 [a, w]
    Vector2d u_k;
    MatrixXd U_result = MatrixXd::Zero(2, N);
    for (int i = 0; i < N; i++) {
        u_k(0) = solution[2 * i];       // 线加速度
        u_k(1) = solution[2 * i + 1];   // 角速度
        U_result.col(i) = u_k;
    }
    
    return U_result;
}

bool MPC_ACC::calculateVelocity(const geometry_msgs::PoseStamped& current_pose, 
                                geometry_msgs::Twist& cmd_vel) {
    std::vector<Eigen::Vector4d> X_r;  // 改为4维状态向量
    std::vector<Eigen::Vector2d> U_r;
    Eigen::MatrixXd u_k;
    Eigen::Vector3d pos_r, pos_r_1, pos_final, v_r_1, v_r_2;
    Eigen::Vector4d X_k;  // 直接定义为4维状态向量
    Eigen::Vector2d u_r;
    Eigen::Vector4d x_r, x_r_1, x_r_2;  // 改为4维状态向量
    double v_linear_1, w;
    double t_k, t_k_1;

    // 直接访问基类的protected成员
    auto discretized_trajectory = trajectory_info_.getTrajectoryPtr();
    if (discretized_trajectory == nullptr) {
        ROS_WARN("No trajectory. Stop!!!");
        return false;
    }
    
    trajectory_utils::TrajectoryPoint traj_point;
    trajectory_info_.getRefTrajectoryPoint(
            trajectory_utils::Vec2d(current_pose.pose.position.x, current_pose.pose.position.y), traj_point);
    double t_cur = traj_point.relative_time();
    
    // 直接访问基类的protected成员
    std::cout << "MPC_ACC t_cur: " << t_cur << ",  traj_duration: " << traj_duration_ << std::endl;

    auto end_point = discretized_trajectory->Evaluate(traj_duration_);
    pos_final << end_point.path_point().x(), end_point.path_point().y(), 0.0;

    double remain_s = end_point.path_point().s() - traj_point.path_point().s();

    if (remain_s < 0.06) {
        current_v_ = 0.0;
        ROS_WARN("MPC_ACC remain_s < 0.06");
        cout << "traj_point: " << traj_point.DebugString() << std::endl;
        cout << "end_point: " << end_point.DebugString() << std::endl;
        return false;
    }

    std::vector<double> t_vec, x_ref_vec, y_ref_vec, theta_ref_vec, s_ref_vec, v_ref_vec, w_ref_vec, a_ref_vec, kappa_ref_vec;

    bool is_orientation_adjust = false;
    bool first_flag = true;
    bool direction = false;
    double orientation_adjust = 0;

    // 构建参考轨迹
    for (int i = 0; i < N_acc_; i++) {
        t_k = t_cur + i * t_step_acc_;
        t_k_1 = t_cur + (i + 1) * t_step_acc_;

        t_vec.push_back(t_k);

        auto pos_r_raw = discretized_trajectory->Evaluate(t_k);
        auto pos_r_1_raw = discretized_trajectory->Evaluate(t_k_1);
        pos_r << pos_r_raw.path_point().x(), pos_r_raw.path_point().y(), 0.0;

        x_r(0) = pos_r(0);
        x_r(1) = pos_r(1);

        x_ref_vec.push_back(pos_r(0));
        y_ref_vec.push_back(pos_r(1));
        s_ref_vec.push_back(pos_r_raw.path_point().s());

        v_linear_1 = pos_r_raw.v();
        v_ref_vec.push_back(v_linear_1);
        x_r(3) = v_linear_1;  // 添加参考速度到状态向量

        x_r(2) = pos_r_raw.path_point().theta();
        theta_ref_vec.push_back(x_r(2));

        double yaw1 = pos_r_raw.path_point().theta();
        double yaw2 = pos_r_1_raw.path_point().theta();

        // 处理角度跳跃
        if (is_orientation_adjust) {
            x_r(2) += orientation_adjust;
        }

        if (abs(yaw2 - yaw1) > M_PI) {
            is_orientation_adjust = true;
            if (first_flag) {
                first_flag = false;
                if ((yaw2 - yaw1) < 0) {
                    direction = true;
                } else {
                    direction = false;
                }
            }
            if (direction) {
                if ((yaw2 - yaw1) >= 0) {
                    is_orientation_adjust = false;
                }
            } else {
                if ((yaw2 - yaw1) < 0) {
                    is_orientation_adjust = false;
                }
            }

            if ((yaw2 - yaw1) < 0) {
                orientation_adjust = 2 * M_PI;
            } else {
                orientation_adjust = -2 * M_PI;
            }
        }

        w = pos_r_raw.v() * pos_r_raw.path_point().kappa();
        w_ref_vec.push_back(w);
        kappa_ref_vec.push_back(pos_r_raw.path_point().kappa());

        u_r(0) = pos_r_raw.a();  // 参考线加速度
        u_r(1) = w;           // 参考角速度

        X_r.push_back(x_r);
        U_r.push_back(u_r);
    }

    // 构造4维当前状态向量 [x, y, theta, v]
    X_k(0) = current_pose.pose.position.x;
    X_k(1) = current_pose.pose.position.y;
    double yaw = tf::getYaw(current_pose.pose.orientation);
    if (yaw / X_r[0](2) < 0 && abs(yaw) > (M_PI * 5 / 6)) {
        if (yaw < 0) {
            X_k(2) = yaw + 2 * M_PI;
        } else {
            X_k(2) = yaw - 2 * M_PI;
        }
    } else {
        X_k(2) = yaw;
    }
    X_k(3) = current_v_;  // 添加当前速度

    // 求解得到加速度控制序列 [a, w]
    u_k = solve(X_k, X_r, U_r, N_acc_);

    // 计算MPC预测轨迹
    calculateMpcTrajectory(X_k, u_k);

    // 从加速度积分得到速度
    double dt = t_step_acc_;
    double new_v = current_v_ + u_k.col(0)(0) * dt;  // v = v0 + a*dt
    double new_w = u_k.col(0)(1);                    // 直接使用求解的角速度

    // 更新当前速度状态用于下次迭代
    current_v_ = new_v;

    // 输出速度命令
    cmd_vel.linear.x = new_v;
    cmd_vel.angular.z = new_w;

    // cmd_vel.angular.x = u_k.col(0)(0); // 用于记录当前参考速度
    // cmd_vel.angular.y = discretized_trajectory->Evaluate(t_cur).v(); // 用于记录当前参考速度
    
    std::cout << "MPC_ACC cmd_vel.linear.x: " << new_v << " m/s, acceleration: " << u_k.col(0)(0) << " m/s²" << "angular vel:" << new_w << std::endl;

    return true;
}

void MPC_ACC::calculateMpcTrajectory(const Eigen::Vector4d& X_k, const Eigen::MatrixXd& u_k) {
    mpc_traj_.clear();
    mpc_traj_.reserve(N_acc_ + 1);  // 预留空间：N个预测步 + 当前状态

    // 添加当前状态作为轨迹起点
    geometry_msgs::PoseStamped current_pose;
    current_pose.header.frame_id = "map";  // 假设使用map坐标系
    current_pose.header.stamp = ros::Time::now();
    current_pose.pose.position.x = X_k(0);
    current_pose.pose.position.y = X_k(1);
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation = tf::createQuaternionMsgFromYaw(X_k(2));
    mpc_traj_.push_back(current_pose);

    // 通过积分控制序列计算预测轨迹
    Eigen::Vector4d state = X_k;  // 当前状态 [x, y, theta, v]

    for (int i = 0; i < N_acc_; i++) {
        // 获取当前步的控制量
        double a_i = u_k(0, i);  // 线加速度
        double w_i = u_k(1, i);  // 角速度

        // 状态更新：使用运动学模型积分
        // dx/dt = v*cos(theta)
        // dy/dt = v*sin(theta)
        // dtheta/dt = w
        // dv/dt = a

        double dt = t_step_acc_;
        double v_current = state(3);
        double theta_current = state(2);

        // 更新状态
        state(0) += v_current * cos(theta_current) * dt;  // x
        state(1) += v_current * sin(theta_current) * dt;  // y
        state(2) += w_i * dt;                             // theta
        state(3) += a_i * dt;                             // v

        cout << "MPC_ACC step " << i << ": a=" << a_i << ", w=" << w_i << ", x:" << ", theta=" << state(2) << ", v=" << state(3) << endl;

        // 角度归一化到 [-π, π]
        while (state(2) > M_PI) state(2) -= 2 * M_PI;
        while (state(2) < -M_PI) state(2) += 2 * M_PI;

        // 构造轨迹点
        geometry_msgs::PoseStamped predicted_pose;
        predicted_pose.header.frame_id = "map";
        predicted_pose.header.stamp = ros::Time::now() + ros::Duration((i + 1) * dt);
        predicted_pose.pose.position.x = state(0);
        predicted_pose.pose.position.y = state(1);
        predicted_pose.pose.position.z = 0.0;
        predicted_pose.pose.orientation = tf::createQuaternionMsgFromYaw(state(2));

        mpc_traj_.push_back(predicted_pose);
    }
}