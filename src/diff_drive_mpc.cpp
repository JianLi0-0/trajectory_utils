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
#include "eigen2cv.hpp"

using namespace Eigen;
using namespace grid_map;

DiffDriveMPC::DiffDriveMPC(int N, double Ts, costmap_2d::Costmap2D* costmap_ptr)
    : N_(N), Ts_(Ts), n_state_(4), n_control_(2),
      last_u_(Vector2d::Zero()), costmap_ptr_(costmap_ptr), sdf_(std::vector<std::string>{"obstacle", "distance"}) {
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
     * =================================================================================================
     * 基于单步离散动力学的一阶CBF安全滤波器 (不使用二阶导数)
     * =================================================================================================
     *
     * 目标: 找到一个安全的控制输入 u_safe = [a, w]^T，它与期望的 u_des 尽可能接近，
     * 同时满足安全约束 h(x_{k+1}) >= (1-gamma)h(x_k)。
     *
     * 1. 安全函数 h(x):
     *    h(x) = d(p) - d_stop = d(p) - v^2 / (2 * a_decel_max)
     *    其中 d(p) 是到障碍物的距离，v 是当前速度，a_decel_max 是最大减速度。
     *    h(x) >= 0 意味着车辆有足够距离在撞到障碍物前停下。
     *
     * 2. 离散CBF约束 (一阶泰勒展开后):
     *    h(x_k) + ∇h(x_k)^T * (x_{k+1} - x_k) >= (1 - gamma) * h(x_k)
     *    => ∇h(x_k)^T * (x_{k+1} - x_k) >= -gamma * h(x_k)
     *
     * 3. 离散动力学 (x_{k+1} - x_k):
     *    Δx = x_{k+1} - x_k = [Ts*v*cos(theta), Ts*v*sin(theta), Ts*w, Ts*a]^T
     *
     * 4. 整合得到关于 u_k = [a, w]^T 的线性不等式:
     *    ∇h(x_k)^T * Δx >= -gamma * h(x_k)
     *    [∇h₃*Ts, ∇h₂*Ts] * [a, w]ᵀ >= -gamma*h_k - [∇h₀, ∇h₁] * [Ts*v*cosθ, Ts*v*sinθ]ᵀ
     *    这可以被构造成 A*u <= b 的形式。
     *
     * 优化问题 (QP):
     *   minimize_{u_safe, s}  ||u_safe - u_des||^2 + rho * s^2
     *
     *   subject to:
     *   1. CBF 安全约束 (含松弛变量 s):
     *      A_cbf * u_safe - s <= b_cbf
     *
     *   2. 控制/速度/松弛变量约束
     * =================================================================================================
     */

    // 1. 获取SDF及其一阶导数
    generateDistanceMap();
    if (sdf_.getLayers().empty()) {
        u_safe = u_des;
        return true;
    }

    grid_map::Position current_pos(state(0), state(1));
    if (!sdf_.isInside(current_pos)) {
        u_safe = u_des;
        return true; // Out of bounds
    }
    double dist = sdf_.atPosition("distance", current_pos);
    double grad_x = sdf_.atPosition("grad_x", current_pos);
    double grad_y = sdf_.atPosition("grad_y", current_pos);


    // 2. 计算 h(x_k) 和 ∇h(x_k)
    double theta = state(2);
    double v = state(3);
    double a_decel_max = 0.8; // 最大减速度 (正值)

    double h_k = dist - (v * v) / (2.0 * a_decel_max);

    Eigen::Vector4d grad_h;
    grad_h(0) = grad_x; // ∂h/∂x = ∂d/∂x
    grad_h(1) = grad_y; // ∂h/∂y = ∂d/∂y
    // 近似 ∂h/∂θ = ∂d/∂θ ≈ (∂p/∂θ) * ∇d
    // ∂p/∂θ = [-v*sin(θ), v*cos(θ)]^T
    grad_h(2) = grad_h(0) * (-v * Ts_ * sin(theta)) + grad_h(1) * (v * Ts_ * cos(theta));
    grad_h(3) = -v / a_decel_max;      // ∂h/∂v

    // 3. 构建 QP 问题
    // min 0.5 * x' * H * x + f' * x, where x = [a, w, s]^T
    int n_vars = 3; // a, w, s
    Eigen::Matrix3d H_qp = Eigen::Matrix3d::Identity();
    H_qp(0,0) = 2.0;
    H_qp(1,1) = 2.0;
    double rho = 1e4; // 松弛变量的惩罚权重 (增大以优先满足安全)
    H_qp(2,2) = 2.0 * rho;

    Eigen::Vector3d f_qp;
    f_qp << -2.0 * u_des(0), -2.0 * u_des(1), 0;

    // 4. 构建约束
    // lower <= A * x <= upper
    int n_constraints = 5; // 1 CBF + 2 control_bounds + 1 velocity_bound + 1 slack_bound
    Eigen::SparseMatrix<double> A_sparse(n_constraints, n_vars);
    Eigen::VectorXd lower_bound(n_constraints);
    Eigen::VectorXd upper_bound(n_constraints);

    std::vector<Eigen::Triplet<double>> triplets;

    // CBF 约束: A_cbf * u - s <= b_cbf
    // A_cbf = -[∇h₃*Ts, ∇h₂*Ts]
    // b_cbf = -(-gamma*h_k - [∇h₀, ∇h₁] * [Ts*v*cosθ, Ts*v*sinθ]ᵀ)
    // 整理为: (∇h₃*Ts)*a + (∇h₂*Ts)*w >= -γ*h_k - (∇h₀*Ts*v*cosθ + ∇h₁*Ts*v*sinθ)
    // 乘以-1得到 <= 形式: -(∇h₃*Ts)*a - (∇h₂*Ts)*w <= γ*h_k + (∇h₀*Ts*v*cosθ + ∇h₁*Ts*v*sinθ)
    double gamma = 1.0; // CBF 增益 (可调)

    double A_cbf_a = -grad_h(3) * Ts_;
    double A_cbf_w = -grad_h(2) * Ts_;
    triplets.emplace_back(0, 0, A_cbf_a); // a的系数
    triplets.emplace_back(0, 1, A_cbf_w); // w的系数
    triplets.emplace_back(0, 2, -1.0);    // s的系数

    double drift_term = grad_h(0) * Ts_ * v * cos(theta) + grad_h(1) * Ts_ * v * sin(theta);
    double b_cbf = gamma * h_k + drift_term;

    lower_bound(0) = -OsqpEigen::INFTY;
    upper_bound(0) = b_cbf;

    // 控制输入约束 (a_min <= a <= a_max, w_min <= w <= w_max)
    triplets.emplace_back(1, 0, 1.0);
    lower_bound(1) = a_min_;
    upper_bound(1) = a_max_;
    triplets.emplace_back(2, 1, 1.0);
    lower_bound(2) = w_min_;
    upper_bound(2) = w_max_;

    // 速度约束 (0 <= v_k + Ts*a <= v_max)
    triplets.emplace_back(3, 0, 1.0);
    lower_bound(3) = (0 - v) / Ts_;
    upper_bound(3) = (v_max_ - v) / Ts_;

    // 松弛变量约束 (s >= 0)
    triplets.emplace_back(4, 2, 1.0);
    lower_bound(4) = 0;
    upper_bound(4) = OsqpEigen::INFTY;

    A_sparse.setFromTriplets(triplets.begin(), triplets.end());

    // 5. OSQP 求解
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
    solver.initSolver();
    solver.solveProblem();
    if (solver.getStatus() != OsqpEigen::Status::Solved) {
        u_safe.setZero();
        return false;
    }

    Eigen::VectorXd solution = solver.getSolution();
    u_safe = solution.head<2>();

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
    cv::distanceTransform(eigen2cv(binary), eigen2cv(sdf_.get("distance")),
                          CV_DIST_L2, CV_DIST_MASK_PRECISE);
    auto map_resolution = costmap_ptr_->getResolution();
    ROS_INFO("map resolution:%f", map_resolution);
    sdf_.get("distance") *= map_resolution;
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

geometry_msgs::PoseArray DiffDriveMPC::getSdfGradientsAsArrows() const {
    return sdf_gradients_;
}
