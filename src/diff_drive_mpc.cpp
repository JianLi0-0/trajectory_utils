#include "diff_drive_mpc.h"
#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <costmap_2d/cost_values.h>
#include <opencv2/opencv.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include <grid_map_costmap_2d/Costmap2DConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/PoseArray.h>
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace grid_map;

DiffDriveMPC::DiffDriveMPC(int N, double Ts, costmap_2d::Costmap2D* costmap_ptr)
    : N_(N), Ts_(Ts), n_state_(4), n_control_(2),
      last_u_(Vector2d::Zero()), costmap_ptr_(costmap_ptr), sdf_(std::vector<std::string>{"obstacle", "distance", "grad_x", "grad_y"}) {
    setConstraints(-2.0, 1.0, -1.0, 1.0, 2.5);
    setWeights(0.5, 1.0, 5.0, 2.0);

    sdf_.setFrameId("map");
    sdf_.setGeometry(grid_map::Length(costmap_ptr_->getSizeInMetersX(), costmap_ptr_->getSizeInMetersY()), costmap_ptr_->getResolution());
}

void DiffDriveMPC::setConstraints(double a_min, double a_max, double w_min, double w_max, double v_max) {
    a_min_ = a_min; a_max_ = a_max;
    w_min_ = w_min; w_max_ = w_max;
    v_max_ = v_max;
}

void DiffDriveMPC::setWeights(double w_a, double w_omega, double w_r, double w_theta) {
    // control weights
    w_a_ = w_a;
    w_omega_ = w_omega;
    // state weights
    w_position_ = w_r;
    w_theta_ = w_theta;
}

bool DiffDriveMPC::solve(const Vector4d &state, const Vector2d &target, double d_des, Vector2d &u_opt) {
    /*
     * =================================================================================================
     * MPC Optimization Problem Formulation
     * =================================================================================================
     *
     * The goal is to find an optimal control sequence U = [u_0, u_1, ..., u_{N-1}] that minimizes a cost
     * function over a prediction horizon N, subject to system dynamics and constraints.
     *
     * State vector:      x = [x, y, theta, v]^T  (position, orientation, velocity)
     * Control vector:    u = [a, w]^T            (acceleration, angular velocity)
     *
     * Cost Function (to be minimized):
     *   J(U) = sum_{k=0}^{N-1} [ (x_k - x_ref)^T * Q * (x_k - x_ref) + u_k^T * R * u_k ] + (p_N - p_ref)^T * Q_N * (p_N - p_ref)
     *
     *   - (x_k - x_ref)^T * Q * (x_k - x_ref): Penalizes deviation from the reference state (target position).
     *   - u_k^T * R * u_k: Penalizes control effort.
     *   - (p_N - p_ref)^T * Q_N * (p_N - p_ref): Penalizes the distance between the final predicted position (p_N)
     *     and a virtual reference point (p_ref), which is at a desired distance 'd_des' from the final target.
     *
     * Subject to the following constraints:
     *
     * 1. System Dynamics (discrete-time nonlinear model):
     *    x_{k+1} = f(x_k, u_k)
     *    x_{k+1}(0) = x_k(0) + Ts * x_k(3) * cos(x_k(2))
     *    x_{k+1}(1) = x_k(1) + Ts * x_k(3) * sin(x_k(2))
     *    x_{k+1}(2) = x_k(2) + Ts * u_k(1)
     *    x_{k+1}(3) = x_k(3) + Ts * u_k(0)
     *
     * 2. Initial State:
     *    x_0 = current_state
     *
     * 3. Control Input Constraints:
     *    a_min <= u_k(0) <= a_max      (for k = 0 to N-1)
     *    w_min <= u_k(1) <= w_max      (for k = 0 to N-1)
     *
     * 4. State Constraints:
     *    0 <= x_k(3) <= v_max          (for k = 1 to N)
     *
     * The problem is then linearized and converted into a Quadratic Programming (QP) problem of the form:
     *
     * Minimize:
     *   1/2 * U^T * H * U + f^T * U
     *
     * Subject to:
     *   l <= A_c * U <= u
     *
     * which is then solved by the OSQP solver.
     * =================================================================================================
     */

    // =========================
    // 1. N步预测模型线性化
    // =========================
    std::vector<MatrixXd> A(N_, MatrixXd::Zero(n_state_, n_state_));
    std::vector<MatrixXd> B(N_, MatrixXd::Zero(n_state_, n_control_));
    std::vector<VectorXd> c(N_, VectorXd::Zero(n_state_));

    Vector4d s_k = state;

    // 计算前馈角速度
    // double angle_to_target = atan2(target(1) - state(1), target(0) - state(0));
    // double angle_error = angle_to_target - state(2);
    // // 将角度误差归一化到[-PI, PI]
    // while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    // while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    // double w_ff = 2.0 * angle_error; // k_p = 2.0 是一个比例增益，可以调整
    // w_ff = std::max(w_min_, std::min(w_max_, w_ff)); // 限制在角速度范围内

    Vector2d u_k = Vector2d::Zero();
    // u_k(1) = w_ff; // 将前馈角速度作为线性化参考

    reference_traj_.clear();
    reference_traj_.resize(N_);
    for(int k=0;k<N_;k++){
        double theta = s_k(2);
        double v = s_k(3);

        // 状态对状态雅可比
        A[k] << 1, 0, -Ts_*v*sin(theta), Ts_*cos(theta),
                0, 1,  Ts_*v*cos(theta), Ts_*sin(theta),
                0, 0, 1, 0,
                0, 0, 0, 1;

        // 状态对控制输入雅可比
        B[k] << 0, 0,
                0, 0,
                0, Ts_,
                Ts_, 0;

        // 线性化偏置项
        Vector4d f;
        f << s_k(0) + Ts_*v*cos(theta),
             s_k(1) + Ts_*v*sin(theta),
             s_k(2) + Ts_*u_k(1),
             s_k(3) + Ts_*u_k(0);
        c[k] = f - A[k]*s_k - B[k]*u_k;

        s_k = f;

        reference_traj_[k].header.frame_id = "map";
        reference_traj_[k].pose.position.x = s_k(0);
        reference_traj_[k].pose.position.y = s_k(1);
        reference_traj_[k].pose.orientation = tf::createQuaternionMsgFromYaw(s_k(2));
    }

    // =========================
    // 2. 构建预测矩阵
    // x_pred = A_bar*s0 + B_bar*U + C_bar
    // =========================
    MatrixXd B_bar = MatrixXd::Zero(N_*n_state_, N_*n_control_);
    MatrixXd A_bar = MatrixXd::Zero(N_*n_state_, n_state_);
    VectorXd C_bar = VectorXd::Zero(N_*n_state_);
    MatrixXd A_prod = MatrixXd::Identity(n_state_, n_state_);

    for(int i=0;i<N_;i++){
        A_bar.block(i*n_state_,0,n_state_,n_state_) = A_prod*A[i];
        for(int j=0;j<=i;j++){
            MatrixXd A_mult = MatrixXd::Identity(n_state_,n_state_);
            for(int k=j+1;k<=i;k++) A_mult *= A[k];
            B_bar.block(i*n_state_, j*n_control_, n_state_, n_control_) = A_mult*B[j];
        }
        VectorXd c_sum = VectorXd::Zero(n_state_);
        if (i > 0) {
            MatrixXd A_temp = MatrixXd::Identity(n_state_, n_state_);
            for(int k=i-1; k>=0; --k) {
                c_sum += A_temp * c[k];
                A_temp *= A[k+1];
            }
        }
        c_sum += c[i];
        C_bar.segment(i*n_state_, n_state_) = c_sum;
        A_prod *= A[i];
    }

    // =========================
    // 3. 构建QP代价
    // J = sum (x_k - x_ref)'*Q*(x_k - x_ref) + u_k'*R*u_k
    // =========================
    MatrixXd Q = MatrixXd::Zero(n_state_, n_state_);
    Q(0,0) = w_position_; // weight for x
    Q(1,1) = w_position_; // weight for y
    Q(2,2) = w_theta_; // weight for theta
    MatrixXd Q_bar = MatrixXd::Zero(N_*n_state_, N_*n_state_);
    for(int i=0; i<N_; ++i) {
        Q_bar.block(i*n_state_, i*n_state_, n_state_, n_state_) = Q;
    }

    MatrixXd R_bar = MatrixXd::Zero(N_*n_control_,N_*n_control_);
    for(int i=0; i<N_; ++i) {
        R_bar(i*2, i*2) = w_a_;
        R_bar(i*2+1, i*2+1) = w_omega_;
    }

    double angle_to_target = atan2(target(1) - state(1), target(0) - state(0));
    // Normalize angle_to_target to be within [-PI, PI] of the current angle
    angle_to_target = state(2) + atan2(sin(angle_to_target - state(2)), cos(angle_to_target - state(2)));

    VectorXd x_ref = VectorXd::Zero(N_*n_state_);
    for(int i=0; i<N_; ++i) {
        x_ref(i*n_state_ + 0) = target(0);
        x_ref(i*n_state_ + 1) = target(1);
        x_ref(i*n_state_ + 2) = angle_to_target;
    }

    MatrixXd H = B_bar.transpose()*Q_bar*B_bar + R_bar;
    VectorXd f = B_bar.transpose()*Q_bar*(A_bar*state + C_bar - x_ref);

    // 增加末端距离惩罚项，使得车辆停在距离目标点 d_des 的位置
    double w_dist_penalty = 1000.1; // 距离惩罚权重，可以调整
    MatrixXd S_N = MatrixXd::Zero(2, N_ * n_state_);
    S_N(0, (N_ - 1) * n_state_ + 0) = 1.0; // 提取预测时域末端的 x 坐标
    S_N(1, (N_ - 1) * n_state_ + 1) = 1.0; // 提取预测时域末端的 y 坐标

    MatrixXd B_N = S_N * B_bar;
    VectorXd A_N_s0_plus_C_N = S_N * (A_bar * state + C_bar);

    // 为了让车辆停在距离目标点 d_des 的位置，我们设定一个虚拟的参考点 p_ref。
    // 该参考点位于当前车辆位置与目标点之间的连线上，且距离目标点为 d_des。
    // 代价函数会惩罚预测的末端位置与这个虚拟参考点之间的距离。
    Vector2d current_pos = state.head<2>();
    Vector2d vec_to_target = target - current_pos;
    Vector2d p_ref = target - vec_to_target.normalized() * d_des;

    // 为代价函数增加末端惩罚项: w_dist * || p_N - p_ref ||^2
    // 其中 p_N = B_N * U + A_N_s0_plus_C_N 是预测的末端位置。
    // 这会更新 QP 问题的 H 矩阵和 f 向量。
    H += 2.0 * w_dist_penalty * B_N.transpose() * B_N;
    f += 2.0 * w_dist_penalty * B_N.transpose() * (A_N_s0_plus_C_N - p_ref);

    // =========================
    // 4. 约束
    // =========================
    // 4.1 控制输入约束 u_lower <= u <= u_upper
    VectorXd u_lower = VectorXd::Constant(N_*n_control_, a_min_);
    VectorXd u_upper = VectorXd::Constant(N_*n_control_, a_max_);
    for(int i=0;i<N_;i++){
        u_lower(i*2+1) = w_min_;
        u_upper(i*2+1) = w_max_;
    }

    // 4.2 速度约束 v >= 0
    MatrixXd Sv = MatrixXd::Zero(N_, N_ * n_state_);
    for(int i=0; i<N_; ++i) {
        Sv(i, i*n_state_ + 3) = 1.0; // 提取速度v
    }
    MatrixXd A_v_constraint = Sv * B_bar;
    VectorXd lower_v_constraint = -Sv * (A_bar * state + C_bar);
    VectorXd upper_v_constraint = VectorXd::Constant(N_, v_max_) - Sv * (A_bar * state + C_bar);

    // 4.3 合并约束
    int n_vars = N_ * n_control_;
    int n_constraints = n_vars + N_;
    SparseMatrix<double> A_sparse(n_constraints, n_vars);
    VectorXd lower_bound(n_constraints);
    VectorXd upper_bound(n_constraints);

    std::vector<Triplet<double>> triplets;
    // 控制约束
    for(int i=0; i<n_vars; ++i) triplets.emplace_back(i, i, 1.0);
    lower_bound.head(n_vars) = u_lower;
    upper_bound.head(n_vars) = u_upper;

    // 速度约束
    for (int i = 0; i < A_v_constraint.rows(); ++i) {
        for (int j = 0; j < A_v_constraint.cols(); ++j) {
            if (A_v_constraint(i, j) != 0) {
                triplets.emplace_back(n_vars + i, j, A_v_constraint(i, j));
            }
        }
    }
    lower_bound.tail(N_) = lower_v_constraint;
    upper_bound.tail(N_) = upper_v_constraint;

    A_sparse.setFromTriplets(triplets.begin(), triplets.end());

    // =========================
    // 5. OSQP求解
    // =========================
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);
    Eigen::SparseMatrix<double> H_sparse = H.sparseView();
    solver.data()->setHessianMatrix(H_sparse);
    solver.data()->setGradient(f);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);
    solver.initSolver();
    solver.solveProblem();
    if(solver.getStatus() != OsqpEigen::Status::Solved){
        return false;
    }

    VectorXd U = solver.getSolution();
    u_opt << U(0), U(1);

    // 用非线性模型计算预测轨迹
    mpc_traj_.clear();
    mpc_traj_.resize(N_);
    Vector4d current_s = state;
    // std::string control_string = "";
    for (int i = 0; i < N_; ++i) {
        Vector2d current_u = U.segment(i * n_control_, n_control_);
        double theta = current_s(2);
        double v = current_s(3);

        Vector4d next_s;
        next_s(0) = current_s(0) + Ts_ * v * cos(theta);
        next_s(1) = current_s(1) + Ts_ * v * sin(theta);
        next_s(2) = current_s(2) + Ts_ * current_u(1);
        next_s(3) = current_s(3) + Ts_ * current_u(0);
        // control_string += "Step " + std::to_string(i) + ": a=" + std::to_string(current_u(0)) +
        //                   ", w=" + std::to_string(current_u(1)) + "\n";

        mpc_traj_[i].header.frame_id = "map";
        mpc_traj_[i].pose.position.x = next_s(0);
        mpc_traj_[i].pose.position.y = next_s(1);
        mpc_traj_[i].pose.orientation = tf::createQuaternionMsgFromYaw(next_s(2));
        current_s = next_s;
    }

    // ROS_WARN_STREAM("MPC Control:\n" << control_string);

    last_u_ = u_opt;
    return true;
}

bool DiffDriveMPC::applySafetyFilter(const Eigen::Vector4d& state, const Eigen::Vector2d& u_des, Eigen::Vector2d& u_safe) {
    /*
     * 功能说明：
     * - 将输入 u_des = [a_des, w_des] 积分为期望线速度 v_des = state.v + a_des * Ts_，期望角速度 w_des = w_des。
     * - 构造带松弛变量 s 的小型 QP，变量为 [v_cmd, w_cmd, s]：
     *     目标：最小化 (v_cmd - v_desired)^2 + (w_cmd - w_desired)^2 + rho_s * s^2 + 方向项线性惩罚
     *     约束：
     *       1) 基于 SDF 梯度的 CBF 线性约束：Ts*(grad·heading)*v_cmd >= -gamma*h - s
     *       2) 线性化的“能停下”约束（利用当前速度处对停止距离线性化），允许以 s 放宽
     *       3) 速度/角速度以及 s 的上下界（s >= 0）
     * - 求解成功后，将第一分量 v_cmd 转换为加速度 a = (v_cmd - state.v) / Ts_，返回 u_safe = [a, w_cmd]。
     *
     * 该实现保证 QP 在不可避免逼近障碍时仍有可行解（通过 s），并通过方向项弱化朝障碍方向的加速倾向。
     */

    // 1) desired control (linear speed form)
    double v_desired = state(3) + u_des(0) * Ts_;
    double w_desired = u_des(1);

    // 2) obtain SDF and gradients
    generateDistanceMap();
    if (sdf_.getLayers().empty()) {
        u_safe << (v_desired - state(3)) / Ts_, w_desired;
        return true;
    }
    grid_map::Position current_pos(state(0), state(1));
    if (!sdf_.isInside(current_pos)) {
        u_safe << (v_desired - state(3)) / Ts_, w_desired;
        return true;
    }
    double dist = sdf_.atPosition("distance", current_pos);
    double grad_x = sdf_.atPosition("grad_x", current_pos);
    double grad_y = sdf_.atPosition("grad_y", current_pos);

    // 3) QP variables: [v_cmd, w_cmd, s]
    int n_vars = 3;
    Eigen::MatrixXd H_qp = 2.0 * Eigen::MatrixXd::Identity(n_vars, n_vars);
    double rho_s = 1e3; // slack penalty (large to prefer safety)
    H_qp(2,2) = 2.0 * rho_s;

    Eigen::VectorXd f_qp(n_vars);
    f_qp << -2.0 * v_desired, -2.0 * w_desired, 0.0;

    // 4) CBF parameters and linearization (correct inequality direction)
    const double d_min = 0.2; // explicit minimum safety distance (meters) — adjust as needed
    const double gamma = 0.5; // CBF shrinkage (tunable, 0<gamma<=1)
    double a_brake = std::abs(a_min_) > 1e-6 ? std::abs(a_min_) : 1.0; // positive braking magnitude
    double v0 = state(3);
    double theta = state(2);

    // current h(x_k) = d - d_min - v0^2/(2*a_brake)
    double h_k = dist - d_min - (v0 * v0) / (2.0 * a_brake);

    // grad · heading
    double grad_dot = grad_x * cos(theta) + grad_y * sin(theta);

    // Conservative linearization:
    //   Δp ≈ Ts * v_cmd * heading  (omit second-order turning term to be conservative)
    //   ∂h/∂v ≈ -v0 / a_brake
    // Discrete CBF requirement:
    //   ∇d·Δp + ∂h/∂v * (v_cmd - v0) >= -gamma * h_k
    //
    // Rearranged into form: coeff_v * v_cmd + coeff_w * w_cmd + 1*s >= rhs_cbf
    double coeff_v = Ts_ * grad_dot + ( - v0 / a_brake ); // multiplies v_cmd
    double coeff_w = 0.0; // omitting turning term (conservative)
    double rhs_cbf = -gamma * h_k + ( - ( -v0 / a_brake ) * v0 ); // move constant terms: note sign algebra
    // simplify rhs_cbf: (-gamma * h_k) - (v0^2 / a_brake)
    rhs_cbf = -gamma * h_k - (v0 * v0) / a_brake;

    // 5) Build sparse constraint matrix and bounds
    // rows:
    // 0: CBF: coeff_v * v_cmd + coeff_w * w_cmd + 1 * s >= rhs_cbf
    // 1: v_cmd >= 0
    // 2: v_cmd <= v_max_
    // 3: w_cmd >= w_min_
    // 4: w_cmd <= w_max_
    // 5: s >= 0
    int n_constraints = 6;
    Eigen::SparseMatrix<double> A_sparse(n_constraints, n_vars);
    std::vector<Eigen::Triplet<double>> triplets;

    triplets.emplace_back(0, 0, coeff_v);
    triplets.emplace_back(0, 1, coeff_w);
    triplets.emplace_back(0, 2, 1.0);

    triplets.emplace_back(1, 0, 1.0);
    triplets.emplace_back(2, 0, 1.0);
    triplets.emplace_back(3, 1, 1.0);
    triplets.emplace_back(4, 1, 1.0);
    triplets.emplace_back(5, 2, 1.0);

    A_sparse.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(n_constraints, -OsqpEigen::INFTY);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Constant(n_constraints, OsqpEigen::INFTY);

    lower_bound(0) = rhs_cbf; upper_bound(0) = OsqpEigen::INFTY;
    lower_bound(1) = 0.0;      upper_bound(1) = OsqpEigen::INFTY;
    lower_bound(2) = -OsqpEigen::INFTY; upper_bound(2) = v_max_;
    lower_bound(3) = w_min_;   upper_bound(3) = OsqpEigen::INFTY;
    lower_bound(4) = -OsqpEigen::INFTY; upper_bound(4) = w_max_;
    lower_bound(5) = 0.0;      upper_bound(5) = OsqpEigen::INFTY;

    // 6) Solve QP
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);
    Eigen::SparseMatrix<double> H_sparse = H_qp.sparseView();
    solver.data()->setHessianMatrix(H_sparse);
    solver.data()->setGradient(f_qp);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    if (!solver.initSolver()) {
        // fallback to desired control
        u_safe << std::max(std::min((v_desired - state(3)) / Ts_, a_max_), a_min_), w_desired;
        return false;
    }

    solver.solveProblem();
    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        u_safe << std::max(std::min((v_desired - state(3)) / Ts_, a_max_), a_min_), w_desired;
        return false;
    }

    Eigen::VectorXd sol = solver.getSolution();
    double v_cmd = sol(0);
    double w_cmd = sol(1);

    double a_cmd = (v_cmd - state(3)) / Ts_;
    if (a_cmd > a_max_) a_cmd = a_max_;
    if (a_cmd < a_min_) a_cmd = a_min_;

    u_safe << a_cmd, w_cmd;
    return true;
}

// New: CBF-based solver that outputs linear velocity and angular velocity (v_cmd, w_cmd)
bool DiffDriveMPC::cbf_solve(const Vector4d &state, const Vector2d &target, Vector2d &u_opt) {
    /*
     * 改进思路：
     * - 增强目标跟踪权重（尤其角速度权重），使到目标的代价高于单纯横向避障。
     * - coeff_w 随距离到目标衰减，距离越远角速度对避障约束影响越小（优先直线前往目标）。
     * - 限幅并缩放避障角速度 w_avoid，远距离时抑制其影响。
     */

    // 1) 计算几何量
    Vector2d current_pos = state.head<2>();
    double dx = target(0) - current_pos(0);
    double dy = target(1) - current_pos(1);
    double dist_to_target = std::hypot(dx, dy);

    // 期望线速度由到目标距离决定
    const double kv = 0.6;
    double v_desired = std::min(v_max_, kv * dist_to_target);

    // 期望角速度由航向误差决定
    double angle_to_target = atan2(dy, dx);
    double angle_error = atan2(sin(angle_to_target - state(2)), cos(angle_to_target - state(2)));
    double w_desired = 1.0 * angle_error;
    w_desired = std::max(w_min_, std::min(w_max_, w_desired));

    // 2) 获取 SDF 与梯度
    generateDistanceMap();
    if (sdf_.getLayers().empty()) {
        u_opt << std::max(0.0, std::min(v_desired, v_max_)), w_desired;
        return false;
    }
    grid_map::Position pos(current_pos(0), current_pos(1));
    if (!sdf_.isInside(pos)) {
        u_opt << std::max(0.0, std::min(v_desired, v_max_)), w_desired;
        return false;
    }
    double dist = sdf_.atPosition("distance", pos);
    double grad_x = sdf_.atPosition("grad_x", pos);
    double grad_y = sdf_.atPosition("grad_y", pos);

    // 额外：靠近障碍时加入避障角速度，但其影响会根据到目标距离被缩放
    const double avoid_dist_thresh = 1.0;
    double w_avoid = 0.0;
    if (dist < avoid_dist_thresh) {
        double avoid_yaw = atan2(-grad_y, -grad_x); // 指向远离障碍的方向
        double yaw_err_avoid = atan2(sin(avoid_yaw - state(2)), cos(avoid_yaw - state(2)));
        const double k_avoid = 1.2;
        double near_factor = (avoid_dist_thresh - dist) / avoid_dist_thresh; // [0,1]
        // 减少远离目标时 avoid 的影响：当离目标较远，降低避障角速度强度
        double target_factor = 1.0 / (1.0 + dist_to_target); // 距离越大，factor 越小
        w_avoid = k_avoid * yaw_err_avoid * near_factor * target_factor;
        // 限幅，防止过大叠加
        w_avoid = std::max(-0.7 * std::abs(w_max_), std::min(0.7 * std::abs(w_max_), w_avoid));
        w_desired += w_avoid;
        w_desired = std::max(w_min_, std::min(w_max_, w_desired));
    }

    // 3) QP 设置：变量 [v_cmd, w_cmd, s]
    const int n_vars = 3;
    Eigen::MatrixXd H_qp = Eigen::MatrixXd::Zero(n_vars, n_vars);

    // 调整权重：增加角速度权重以抑制过度转向；角速度权重随到目标距离增加（约在 [5,50]）
    double w_v = 20.0; // 线速度跟踪权重（保持足够大）
    double w_w_base = 5.0;
    double w_w = w_w_base + 10.0 * (dist_to_target / (dist_to_target + 1.0)); // 约在 [5,50]
    double rho_s = 1e3;

    H_qp(0,0) = 2.0 * w_v;
    H_qp(1,1) = 2.0 * w_w;
    H_qp(2,2) = 2.0 * rho_s;

    Eigen::VectorXd f_qp(n_vars);
    f_qp << -2.0 * w_v * v_desired, -2.0 * w_w * w_desired, 0.0;

    // 4) CBF 约束（加入角速度项，但 coeff_w 随到目标距离衰减）
    const double d_min = 0.6;
    const double gamma = 0.5;
    double theta = state(2);

    double grad_dot = grad_x * cos(theta) + grad_y * sin(theta);
    double grad_perp = -grad_x * sin(theta) + grad_y * cos(theta);

    const double L = 0.4;
    double coeff_v = Ts_ * grad_dot;
    double coeff_w = Ts_ * L * grad_perp;

    // 当离目标较远时，衰减角速度对约束的影响，避免优先横向避障
    double decay = 1.0 + dist_to_target; // 距离越远衰减越强
    coeff_w *= (1.0 / decay);

    double rhs = -gamma * (dist - d_min);

    // 5) 构建约束矩阵与上下界
    const int n_constraints = 6;
    Eigen::SparseMatrix<double> A_sparse(n_constraints, n_vars);
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.emplace_back(0, 0, coeff_v);
    triplets.emplace_back(0, 1, coeff_w);
    triplets.emplace_back(0, 2, 1.0);
    triplets.emplace_back(1, 0, 1.0); // v_cmd >= 0
    triplets.emplace_back(2, 0, 1.0); // v_cmd <= v_max
    triplets.emplace_back(3, 1, 1.0); // w_cmd >= w_min
    triplets.emplace_back(4, 1, 1.0); // w_cmd <= w_max
    triplets.emplace_back(5, 2, 1.0); // s >= 0
    A_sparse.setFromTriplets(triplets.begin(), triplets.end());

    Eigen::VectorXd lower_bound = Eigen::VectorXd::Constant(n_constraints, -OsqpEigen::INFTY);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Constant(n_constraints, OsqpEigen::INFTY);

    lower_bound(0) = rhs;              upper_bound(0) = OsqpEigen::INFTY;
    lower_bound(1) = 0.0;              upper_bound(1) = OsqpEigen::INFTY;
    lower_bound(2) = -OsqpEigen::INFTY; upper_bound(2) = v_max_;
    lower_bound(3) = w_min_;           upper_bound(3) = OsqpEigen::INFTY;
    lower_bound(4) = -OsqpEigen::INFTY; upper_bound(4) = w_max_;
    lower_bound(5) = 0.0;              upper_bound(5) = OsqpEigen::INFTY;

    // 6) 求解 QP
    OsqpEigen::Solver solver;
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_vars);
    solver.data()->setNumberOfConstraints(n_constraints);
    Eigen::SparseMatrix<double> H_sparse = H_qp.sparseView();
    solver.data()->setHessianMatrix(H_sparse);
    solver.data()->setGradient(f_qp);
    solver.data()->setLinearConstraintsMatrix(A_sparse);
    solver.data()->setLowerBound(lower_bound);
    solver.data()->setUpperBound(upper_bound);

    if (!solver.initSolver()) {
        double v_fb = std::max(0.0, std::min(v_desired, v_max_));
        double w_fb = std::max(w_min_, std::min(w_desired, w_max_));
        u_opt << v_fb, w_fb;
        return false;
    }

    solver.solveProblem();
    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        double v_fb = std::max(0.0, std::min(v_desired, v_max_));
        double w_fb = std::max(w_min_, std::min(w_desired, w_max_));
        u_opt << v_fb, w_fb;
        return false;
    }

    Eigen::VectorXd sol = solver.getSolution();
    double v_cmd = sol(0);
    double w_cmd = sol(1);

    // clamp final outputs
    v_cmd = std::max(0.0, std::min(v_cmd, v_max_));
    w_cmd = std::max(w_min_, std::min(w_cmd, w_max_));

    u_opt << v_cmd, w_cmd;
    return true;
}

void DiffDriveMPC::generateDistanceMap() {
    sdf_.setGeometry(grid_map::Length(costmap_ptr_->getSizeInMetersX(), costmap_ptr_->getSizeInMetersY()),
                          costmap_ptr_->getResolution());
    sdf_.setPosition(grid_map::Position(costmap_ptr_->getOriginX()+costmap_ptr_->getSizeInMetersX()/2.0,
                                             costmap_ptr_->getOriginY()+costmap_ptr_->getSizeInMetersY()/2.0));
    // 添加层到gridMap
    grid_map::Costmap2DConverter<grid_map::GridMap,
            grid_map::Costmap2DDirectTranslationTable> costmap2d_converter;
    costmap2d_converter.addLayerFromCostmap2D(*costmap_ptr_, "obstacle", sdf_);

    // 获取obstacle层的Eigen矩阵引用
    auto& obstacle_layer = sdf_["obstacle"];
    // 0变255，其它变0; ros无障碍物时为0, 此处无障碍物时为255，有障碍物时为0，所以需要转换
    obstacle_layer = (obstacle_layer.array() == 0).cast<float>() * 255.f;

    // Update distance layer.
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> binary =
            sdf_.get("obstacle").cast<unsigned char>();
    cv::Mat binary_cv;
    cv::eigen2cv(binary, binary_cv);
    cv::Mat distance_cv;
    cv::distanceTransform(binary_cv, distance_cv, CV_DIST_L2, CV_DIST_MASK_PRECISE);
    cv::cv2eigen(distance_cv, sdf_.get("distance"));

    // 计算距离图的梯度 - 修正坐标系问题，不使用eigen2cv函数
    cv::Mat grad_x_cv, grad_y_cv;
    
    // 使用Sobel算子计算梯度
    cv::Sobel(distance_cv, grad_y_cv, CV_32F, 1, 0, 3); // x方向梯度 -> grid_map的y方向
    cv::Sobel(distance_cv, grad_x_cv, CV_32F, 0, 1, 3); // y方向梯度 -> grid_map的x方向

    auto map_resolution = costmap_ptr_->getResolution();
    sdf_.get("distance") *= map_resolution;

    // 转换为Eigen矩阵并直接赋值
    cv::cv2eigen(grad_x_cv, sdf_.get("grad_x"));
    cv::cv2eigen(grad_y_cv, sdf_.get("grad_y"));

    sdf_.get("grad_x") *= -1;
    sdf_.get("grad_y") *= -1;
}

nav_msgs::OccupancyGrid DiffDriveMPC::getSdfAsOccupancyGrid() const {
    nav_msgs::OccupancyGrid occupancy_grid;
    if (sdf_.exists("distance")) {
        float min_val = sdf_.get("distance").minCoeffOfFinites();
        float max_val = sdf_.get("distance").maxCoeffOfFinites();
        grid_map::GridMapRosConverter::toOccupancyGrid(sdf_, "distance", min_val, max_val, occupancy_grid);
    }
    return occupancy_grid;
}

nav_msgs::OccupancyGrid DiffDriveMPC::getGradXAsOccupancyGrid() const {
    nav_msgs::OccupancyGrid occupancy_grid;
    if (sdf_.exists("grad_x")) {
        float min_val = sdf_.get("grad_x").minCoeffOfFinites();
        float max_val = sdf_.get("grad_x").maxCoeffOfFinites();
        grid_map::GridMapRosConverter::toOccupancyGrid(sdf_, "grad_x", min_val, max_val, occupancy_grid);
    }
    return occupancy_grid;
}

nav_msgs::OccupancyGrid DiffDriveMPC::getGradYAsOccupancyGrid() const {
    nav_msgs::OccupancyGrid occupancy_grid;
    if (sdf_.exists("grad_y")) {
        float min_val = sdf_.get("grad_y").minCoeffOfFinites();
        float max_val = sdf_.get("grad_y").maxCoeffOfFinites();
        grid_map::GridMapRosConverter::toOccupancyGrid(sdf_, "grad_y", min_val, max_val, occupancy_grid);
    }
    return occupancy_grid;
}

geometry_msgs::PoseArray DiffDriveMPC::getSdfGradientsAsArrows() const {
    geometry_msgs::PoseArray pose_array;
    pose_array.header.frame_id = "map";
    pose_array.header.stamp = ros::Time::now();
    
    // Check if gradient layers exist
    if (!sdf_.exists("grad_x") || !sdf_.exists("grad_y")) {
        ROS_WARN("Gradient layers do not exist in SDF");
        return pose_array;
    }
    
    // Sampling parameters
    int skip_cells = 5; // Skip every N cells to reduce arrow density
    double min_gradient_magnitude = 0.1; // Minimum gradient magnitude to display arrow
    
    // Get grid map properties
    const auto& grad_x_layer = sdf_.get("grad_x");
    const auto& grad_y_layer = sdf_.get("grad_y");
    
    // Iterate through the grid map at sampled intervals
    for (grid_map::GridMapIterator iterator(sdf_); !iterator.isPastEnd(); ++iterator) {
        const grid_map::Index index = *iterator;
        
        // Skip cells to reduce density
        // if (index(0) % skip_cells != 0 || index(1) % skip_cells != 0) {
        //     continue;
        // }
        
        // Get gradient values at this cell
        double grad_x = grad_x_layer(index(0), index(1));
        double grad_y = grad_y_layer(index(0), index(1));
        
        // Skip if gradient magnitude is too small
        double gradient_magnitude = sqrt(grad_x * grad_x + grad_y * grad_y);
        if (gradient_magnitude < min_gradient_magnitude) {
            continue;
        }
        
        // Convert grid index to world position
        grid_map::Position world_position;
        sdf_.getPosition(index, world_position);
        
        // Create pose for arrow
        geometry_msgs::Pose arrow_pose;
        arrow_pose.position.x = world_position(0);
        arrow_pose.position.y = world_position(1);
        arrow_pose.position.z = 0.0;
        
        // Calculate orientation from gradient direction
        double arrow_yaw = atan2(grad_y, grad_x);
        arrow_pose.orientation = tf::createQuaternionMsgFromYaw(arrow_yaw);
        
        pose_array.poses.push_back(arrow_pose);
    }
    
    ROS_DEBUG("Generated %lu gradient arrows from grid_map", pose_array.poses.size());
    return pose_array;
}
