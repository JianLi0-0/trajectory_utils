#include "mpc.h"
#include "math.h"
#include "iostream"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <ros/ros.h>

#define PI 3.1415926
#define MPC_TIME_STEP 0.025
#define w0 1.0
#define w1 0.5
#define Ku 1
#define Kl 1


using namespace Eigen;
using namespace std;

#ifdef HAVE_QPOASES
#include <qpOASES.hpp>
using namespace qpOASES;
#endif

MatrixXd MPC::solve(
        Eigen::Vector3d X_k, std::vector<Eigen::Vector3d> X_r,
        std::vector<Eigen::Vector2d> U_r, const int N) {
    auto start = std::chrono::high_resolution_clock::now();
    ////根据参考输入计算出的系数矩阵
    vector<MatrixXd> A_r(N), B_r(N), A_multiply1(N);
    MatrixXd O_r(3 * N, 1);
    MatrixXd A_bar(3 * N, 3);
    MatrixXd X_ref(3 * N, 1);
    MatrixXd A_multiply2;
    MatrixXd B_bar = MatrixXd::Zero(3 * N, 2 * N);
    MatrixXd C_bar = MatrixXd::Identity(3 * N, 3 * N);
    MatrixXd A_r_init = MatrixXd::Zero(3, 3);
    MatrixXd B_r_init = MatrixXd::Zero(3, 2);
    MatrixXd eye_3 = MatrixXd::Identity(3, 3);
    MatrixXd Q = MatrixXd::Identity(3 * N, 3 * N) * omega0_;
    MatrixXd R = MatrixXd::Identity(2 * N, 2 * N);
    // 对R矩阵进行间隔赋值
    for (int i = 0; i < 2 * N; i++) {
        if (i % 2 == 0) {
            R(i, i) = omega1_v_;
        } else {
            R(i, i) = omega1_w_;
        }
    }

    for (int k = 0; k < N; k++) {
        A_r[k] = A_r_init;
        B_r[k] = B_r_init;
        A_r[k](0, 2) = -U_r[k](0) * sin(X_r[k](2));
        A_r[k](1, 2) = U_r[k](0) * cos(X_r[k](2));
        Vector3d temp_vec = -MPC_TIME_STEP * A_r[k] * X_r[k];
        O_r.block<3, 1>(k * 3, 0) = temp_vec;
        A_r[k] = eye_3 + MPC_TIME_STEP * A_r[k];
        B_r[k](0, 0) = cos(X_r[k](2)) * MPC_TIME_STEP;
        B_r[k](1, 0) = sin(X_r[k](2)) * MPC_TIME_STEP;
        B_r[k](2, 1) = MPC_TIME_STEP;
        X_ref.block<3, 1>(k * 3, 0) = X_r[k];

        if (k == 0) A_multiply1[k] = A_r[k];
        else A_multiply1[k] = A_multiply1[k - 1] * A_r[k];
        A_bar.block<3, 3>(3 * k, 0) = A_multiply1[k];
    }

    for (int k = 0; k < N; k++) {
        B_bar.block<3, 2>(3 * k, 2 * k) = B_r[k];
        A_multiply2 = eye_3;
        for (int i = 0; i < k; i++) {
            A_multiply2 = A_multiply2 * A_r[k - i];
            C_bar.block<3, 3>(3 * k, 3 * (k - 1 - i)) = A_multiply2;
            B_bar.block<3, 2>(3 * k, 2 * (k - 1 - i)) = A_multiply2 * B_r[k - 1 - i];
        }
    }

    MatrixXd E = A_bar * X_k + C_bar * O_r - X_ref;
    MatrixXd Hesse = 2 * (B_bar.transpose() * Q * B_bar + R);      ////Hesse矩阵
    VectorXd gradient = 2 * B_bar.transpose() * Q * E;       ////一次项系数

    std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "Problem formulation time taken: " << elapsed.count() << " ms" << std::endl;

#ifdef HAVE_QPOASES
    start = std::chrono::high_resolution_clock::now();
    real_t H[2 * N * 2 * N], g[2 * N], A[2 * N], lb[2 * N], ub[2 * N], lbA[1], ubA[1];
    lbA[0] = N * (v_min_ + w_min_) / Ku;
    ubA[0] = N * (v_max_ + w_max_) / Kl;
    for (int i = 0; i < 2 * N; i++) {
        g[i] = gradient(i);
        A[i] = 1;
        if (i % 2 == 0) {
            lb[i] = v_min_;
            ub[i] = v_max_;
        } else {
            lb[i] = w_min_;
            ub[i] = w_max_;
        }
        for (int j = 0; j < 2 * N; j++) {
            H[i * 2 * N + j] = Hesse(i, j);
        }
    }

    int_t nWSR = 800;

    // 使用SQProblem替代QProblem，更适合MPC问题
    static bool first_qp_call = true;
    // 将求解器定义为静态变量，保持在函数调用之间的状态
    static SQProblem* mpc_qp_solver = nullptr;

    // 第一次调用时创建求解器实例
    if (mpc_qp_solver == nullptr || mpc_qp_solver->getNV() != 2*N) {
        // 如果求解器不存在或问题规模变化，释放旧的并创建新的
        if (mpc_qp_solver != nullptr) {
            delete mpc_qp_solver;
            first_qp_call = true; // 需要重新初始化
        }
        mpc_qp_solver = new SQProblem(2 * N, 1);

        // 设置MPC专用选项，对实时控制系统性能非常重要
        Options options;
        options.setToMPC();
        options.printLevel = PL_LOW;
        mpc_qp_solver->setOptions(options);
    }

    if (first_qp_call) {
        // 第一次调用，使用普通init
        mpc_qp_solver->init(H, g, A, lb, ub, lbA, ubA, nWSR);
        first_qp_call = false;
    } else {
        // 使用SQProblem的hotstart方法，它允许在每次迭代中更新Hessian矩阵(H)和约束矩阵(A)
        mpc_qp_solver->hotstart(H, g, A, lb, ub, lbA, ubA, nWSR);
    }

    real_t x_solution[2 * N];
    mpc_qp_solver->getPrimalSolution(x_solution);

    elapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "qpOASES time taken: " << elapsed.count() << " ms" << std::endl;

    Vector2d u_k;
    MatrixXd U_result = MatrixXd::Zero(2, N);
    for (int i = 0; i < N; i++) {
        u_k(0) = x_solution[2 * i];
        u_k(1) = x_solution[2 * i + 1];
        U_result.col(i) = u_k;
    }

#else
    start = std::chrono::high_resolution_clock::now();
    // 创建OSQP求解器实例
    OsqpEigen::Solver solver;

    // 设置求解器参数
    solver.settings()->setVerbosity(true);   // 输出求解过程信息
    solver.settings()->setWarmStart(true);   // 启用热启动

    Eigen::SparseMatrix<double> sparse_H(Hesse.sparseView());
    VectorXf q = gradient.cast<float>();

    Eigen::SparseMatrix<double> sparse_A(2 * N, 2 * N);
    sparse_A.setIdentity();

    VectorXd lower_bound(2 * N);
    VectorXd upper_bound(2 * N);
    for (int i = 0; i < 2 * N; i++) {
        if (i % 2 == 0) {
            lower_bound(i) = v_min_;
            upper_bound(i) = v_max_;
        } else {
            lower_bound(i) = w_min_;
            upper_bound(i) = w_max_;
        }
    }

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(false);

    // 设置问题数据
    solver.data()->setNumberOfVariables(2*N);
    solver.data()->setNumberOfConstraints(2*N);
    if (!solver.data()->setHessianMatrix(sparse_H)) std::cout << "Problem failed to setHessianMatrix !" << std::endl;;
    if (!solver.data()->setGradient(gradient)) std::cout << "Problem failed to setGradient !" << std::endl;;
    if (!solver.data()->setLinearConstraintsMatrix(sparse_A)) std::cout << "Problem failed to setLinearConstraintsMatrix !" << std::endl;;
    if (!solver.data()->setLowerBound(lower_bound)) std::cout << "Problem failed to setLowerBound !" << std::endl;;
    if (!solver.data()->setUpperBound(upper_bound)) std::cout << "Problem failed to setUpperBound !" << std::endl;;

    // 初始化求解器
    if (!solver.initSolver()) cout << "Problem failed to initSolver !" << std::endl;
    Eigen::VectorXd solution;
    // 执行求解
    if (solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError) {
        // 获取最优解
        solution = solver.getSolution();
//        std::cout << "Optimal solution:\n" << solution << std::endl;
    } else {
        std::cout << "Problem failed to solve!" << std::endl;
    }

    elapsed = std::chrono::high_resolution_clock::now() - start;
    std::cout << "OSQP Time taken: " << elapsed.count() << " ms" << std::endl;

    Vector2d u_k;
    MatrixXd U_result = MatrixXd::Zero(2, N);
    for (int i = 0; i < N; i++) {
        u_k(0) = solution[2 * i];
        u_k(1) = solution[2 * i + 1];
        U_result.col(i) = u_k;
    }

#endif

    return U_result;
}

void MPC::generateReferenceTrajectory(const geometry_msgs::PoseStamped& current_pose,
                                 const std::vector<geometry_msgs::PoseStamped>& path, const double& initial_linear_vel) {
    auto position = current_pose.pose.position;
    trajectory_utils::TrajectoryPoint traj_point;
    if (!trajectory_info_.getRefTrajectoryPoint(
            trajectory_utils::Vec2d(position.x, position.y), traj_point)) {
        traj_point.set_v(initial_linear_vel);
        ROS_WARN("traj_point.v(): %f", traj_point.v());
    }

    std::cout << traj_point.DebugString() << std::endl;

    std::vector<trajectory_utils::PathPoint> path_data;
    for (auto const& pose_stamped : path) {
        trajectory_utils::PathPoint p;
        p.set_x(pose_stamped.pose.position.x);
        p.set_y(pose_stamped.pose.position.y);
        path_data.push_back(p);
    }

    trajectory_info_.setPathData(path_data);

    ROS_WARN("length: %f, save_dist: %f", trajectory_info_.getPathDataPtr()->Length(), save_distance_);

    trajectory_info_.calSpeedData(
            0.0, traj_point.v(), traj_point.a(),
            trajectory_info_.getPathDataPtr()->Length()-save_distance_, ref_v_max_);

    trajectory_info_.combinePathAndSpeedProfile();

    // trajectory_info_.displayTrajProfile();

    traj_duration_ = trajectory_info_.getSpeedDataPtr()->get_duration();
}

bool MPC::calculateVelocity(const geometry_msgs::PoseStamped& current_pose, geometry_msgs::Twist& cmd_vel) {
    std::vector<Eigen::Vector3d> X_r;
    std::vector<Eigen::Vector2d> U_r;
    Eigen::MatrixXd u_k;
    Eigen::Vector3d pos_r, pos_r_1, pos_final, v_r_1, v_r_2, X_k;
    Eigen::Vector2d u_r;
    Eigen::Vector3d x_r, x_r_1, x_r_2;
    double v_linear_1, w;
    double t_k, t_k_1;

    auto discretized_trajectory = trajectory_info_.getTrajectoryPtr();
    if (discretized_trajectory == nullptr) {
        ROS_WARN("No trajectory. Stop!!!");
        return false;
    }
    trajectory_utils::TrajectoryPoint traj_point;
    trajectory_info_.getRefTrajectoryPoint(
            trajectory_utils::Vec2d(current_pose.pose.position.x, current_pose.pose.position.y), traj_point);
    double t_cur = traj_point.relative_time();
    std::cout << "t_cur: " << t_cur << ",  traj_duration_: " << traj_duration_ << std::endl;

    auto end_point = discretized_trajectory->Evaluate(traj_duration_);
    pos_final << end_point.path_point().x(), end_point.path_point().y(), 0.0;

    double remain_s = end_point.path_point().s() - traj_point.path_point().s();

    if (remain_s < 0.06) {
        ROS_WARN("remain_s < 0.06");
        cout << "traj_point: " << traj_point.DebugString() << std::endl;
        cout << "end_point: " << end_point.DebugString() << std::endl;
//        trajectory_info_.reset();
        return false;
    }

    std::vector<double> t_vec, x_ref_vec, y_ref_vec, theta_ref_vec, s_ref_vec, v_ref_vec, w_ref_vec, a_ref_vec, kappa_ref_vec;

    bool is_orientation_adjust = false;
    bool first_flag = true;
    bool direction = false;
    double orientation_adjust=0;

    for (int i = 0; i < N; i++) {

        t_k = t_cur + i * t_step;
        t_k_1 = t_cur + (i + 1) * t_step;

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

        x_r(2) = pos_r_raw.path_point().theta();
        theta_ref_vec.push_back(x_r(2));

        double yaw1 = pos_r_raw.path_point().theta();
        double yaw2 = pos_r_1_raw.path_point().theta();

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
            }
            else {
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

        u_r(0) = v_linear_1;
        u_r(1) = w;

        X_r.push_back(x_r);
        U_r.push_back(u_r);
    }

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

    u_k = solve(X_k, X_r, U_r, N);

    // 计算MPC预测轨迹
    // calculateMpcTrajectory(X_k, u_k);
    // cmd_vel.linear.z = discretized_trajectory->Evaluate(t_cur).v(); // 用于记录当前参考速度

    cmd_vel.linear.x = u_k.col(0)(0);


    cmd_vel.angular.z = u_k.col(0)(1);
    std::cout << "cmd_vel.linear.x : : " << u_k.col(0)(0) << "m/s" << std::endl;

    // trajectory_info_.displayUpdate(t_cur+t_step, cmd_vel.linear.x);

    return true;
}

void MPC::calculateMpcTrajectory(const Eigen::Vector3d& X_k, const Eigen::MatrixXd& u_k) {
    mpc_traj_.clear();
    mpc_traj_.reserve(N + 1);  // 预留空间：N个预测步 + 当前状态

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
    Eigen::Vector3d state = X_k;  // 当前状态 [x, y, theta]

    for (int i = 0; i < N; i++) {
        // 获取当前步的控制量
        double v_i = u_k(0, i);  // 线速度
        double w_i = u_k(1, i);  // 角速度

        std::cout << "v_i: " << v_i << std::endl;

        // 状态更新：使用运动学模型积分
        // dx/dt = v*cos(theta)
        // dy/dt = v*sin(theta)
        // dtheta/dt = w

        double dt = t_step;
        double theta_current = state(2);

        // 更新状态
        state(0) += v_i * cos(theta_current) * dt;  // x
        state(1) += v_i * sin(theta_current) * dt;  // y
        state(2) += w_i * dt;                       // theta

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
