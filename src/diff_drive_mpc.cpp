#include "diff_drive_mpc.h"
#include <OsqpEigen/OsqpEigen.h>
#include <vector>
#include <cmath>
#include <tf/transform_datatypes.h>

using namespace Eigen;

DiffDriveMPC::DiffDriveMPC(int N, double Ts)
    : N_(N), Ts_(Ts), n_state_(4), n_control_(2),
      w_state_(1.0), w_input_(0.1), w_r_(10.0), w_theta_(5.0),
      a_min_(-1.0), a_max_(1.0), w_min_(-1.0), w_max_(1.0), v_max_(2.0),
      last_u_(Vector2d::Zero()), mpc_traj_() {
    setConstraints(-2.0, 1.0, -1.0, 1.0, 2.5);
    setWeights(1.0, 0.5, 5.0, 2.0);
}

void DiffDriveMPC::setConstraints(double a_min, double a_max, double w_min, double w_max, double v_max) {
    a_min_ = a_min; a_max_ = a_max;
    w_min_ = w_min; w_max_ = w_max;
    v_max_ = v_max;
}

void DiffDriveMPC::setWeights(double w_state, double w_input, double w_r, double w_theta) {
    w_state_ = w_state; w_input_ = w_input;
    w_r_ = w_r; w_theta_ = w_theta;
}

bool DiffDriveMPC::solve(const Vector4d &state, const Vector2d &target, double d_des, Vector2d &u_opt) {
    // =========================
    // 1. N步预测模型线性化
    // =========================
    std::vector<MatrixXd> A(N_, MatrixXd::Zero(n_state_, n_state_));
    std::vector<MatrixXd> B(N_, MatrixXd::Zero(n_state_, n_control_));
    std::vector<VectorXd> c(N_, VectorXd::Zero(n_state_));

    Vector4d s_k = state;

    // 计算前馈角速度
    double angle_to_target = atan2(target(1) - state(1), target(0) - state(0));
    double angle_error = angle_to_target - state(2);
    // 将角度误差归一化到[-PI, PI]
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    double w_ff = 2.0 * angle_error; // k_p = 2.0 是一个比例增益，可以调整
    w_ff = std::max(w_min_, std::min(w_max_, w_ff)); // 限制在角速度范围内

    Vector2d u_k = Vector2d::Zero();
    // u_k(1) = w_ff; // 将前馈角速度作为线性化参考

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
    Q(0,0) = w_r_; // weight for x
    Q(1,1) = w_r_; // weight for y
    MatrixXd Q_bar = MatrixXd::Zero(N_*n_state_, N_*n_state_);
    for(int i=0; i<N_; ++i) {
        Q_bar.block(i*n_state_, i*n_state_, n_state_, n_state_) = Q;
    }

    MatrixXd R_bar = MatrixXd::Identity(N_*n_control_,N_*n_control_)*w_input_;

    VectorXd x_ref = VectorXd::Zero(N_*n_state_);
    for(int i=0; i<N_; ++i) {
        x_ref(i*n_state_ + 0) = target(0);
        x_ref(i*n_state_ + 1) = target(1);
    }

    MatrixXd H = B_bar.transpose()*Q_bar*B_bar + R_bar;
    VectorXd f = B_bar.transpose()*Q_bar*(A_bar*state + C_bar - x_ref);

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
