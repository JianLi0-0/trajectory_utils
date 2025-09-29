//
// Created by desmond on 2025/9/28.
//

#ifndef ADMM_QP_H
#define ADMM_QP_H

#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <vector>

using Eigen::VectorXd;
using Eigen::SparseMatrix;
using Eigen::SimplicialLLT;

// ADMM-based QP solver using sparse Cholesky factorization and dynamic rho update
class AdmmQpSolver {
public:
    // Problem data (QP: min 0.5 x^T P x + q^T x, s.t. l <= Ax <= u)
    SparseMatrix<double> P_matrix_;
    VectorXd q_vector_;
    SparseMatrix<double> A_matrix_;
    VectorXd lA_vector_, uA_vector_;

    // ADMM parameters
    double sigma_value_ = 1e-6;  // regularization for numerical stability
    double rho_value_ = 0.1;
    double rho_max_ = 1e6;
    double rho_min_ = 1e-6;
    double alpha_value_ = 1.6;
    double eps_abs_value_ = 1e-3;
    double eps_rel_value_ = 1e-3;
    int max_iter_value_ = 4000;

    // ADMM variables
    VectorXd x_vector_, z_vector_, u_vector_;

    // Factorization cache
    SimplicialLLT<SparseMatrix<double>> kkt_solver_;

    AdmmQpSolver(const SparseMatrix<double>& P, const VectorXd& q,
                  const SparseMatrix<double>& A, const VectorXd& l, const VectorXd& u)
        : P_matrix_(P), q_vector_(q), A_matrix_(A), lA_vector_(l), uA_vector_(u) {
        int n = P.rows();
        int m = A.rows();
        x_vector_ = VectorXd::Zero(n);
        z_vector_ = VectorXd::Zero(m);
        u_vector_ = VectorXd::Zero(m);
        GetStartPoint();
        SetupKkt();  // Precompute Cholesky factorization
    }

    void GetStartPoint(double ls_reg = 1e-8) {
        int n = P_matrix_.rows();
        int m = A_matrix_.rows();

        // 目标 z0 取区间中点（无穷界处理：若存在 +/-inf 则用 0 或单侧界）
        VectorXd z0(m);
        for (int i = 0; i < m; ++i) {
            double li = lA_vector_[i];
            double ui = uA_vector_[i];
            if (std::isinf(li) && std::isinf(ui))      z0[i] = 0.0;
            else if (std::isinf(li))                   z0[i] = std::min(0.0, ui);
            else if (std::isinf(ui))                   z0[i] = std::max(0.0, li);
            else                                       z0[i] = 0.5 * (li + ui);
        }

        // 解 (A^T A + ls_reg I) x = A^T z0  得到 x0
        Eigen::SparseMatrix<double> At = A_matrix_.transpose();
        Eigen::SparseMatrix<double> ATA = At * A_matrix_;
        Eigen::SparseMatrix<double> I(n,n); I.setIdentity();
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double>> ls_solver;
        ls_solver.compute(ATA + ls_reg * I);
        if (ls_solver.info() == Eigen::Success) {
            x_vector_ = ls_solver.solve(At * z0);
        } else {
            x_vector_.setZero();
        }

        // 生成 z, u
        VectorXd Ax = A_matrix_ * x_vector_;
        z_vector_ = Project(Ax);  // 使其落入区间
        u_vector_.setZero();
    }

    // Setup and factorize KKT matrix: K = P + rho * A^T A
    void SetupKkt() {
        SparseMatrix<double> At = A_matrix_.transpose();
        SparseMatrix<double> K_matrix = P_matrix_ + rho_value_ * At * A_matrix_;

        Eigen::SparseMatrix<double> I(P_matrix_.rows(), P_matrix_.cols());
        I.setIdentity();
        K_matrix = K_matrix + sigma_value_ * I;

        kkt_solver_.compute(K_matrix);
        if (kkt_solver_.info() != Eigen::Success) {
            throw std::runtime_error("Cholesky factorization failed.");
        }
    }

    // Project a vector onto the bound constraints [l, u]
    VectorXd Project(const VectorXd& v) {
        VectorXd out(v.size());
        for (int i = 0; i < v.size(); i++) {
            if (v[i] < lA_vector_[i]) out[i] = lA_vector_[i];
            else if (v[i] > uA_vector_[i]) out[i] = uA_vector_[i];
            else out[i] = v[i];
        }
        return out;
    }

    // Main ADMM solver loop with dynamic rho update every check_interval iterations
    void Solve() {
        int n = P_matrix_.rows();
        int m = A_matrix_.rows();
        VectorXd z_old = z_vector_;

        double mu = 10.0;        // threshold for imbalance
        double tau_incr = 2.0;   // increase factor
        double tau_decr = 2.0;   // decrease factor
        int check_interval = 25;  // check residual ratio every 25 iterations

        for (int k = 0; k < max_iter_value_; k++) {
            // x-update: solve KKT system
            VectorXd rhs = -q_vector_ + rho_value_ * A_matrix_.transpose() * (z_vector_ - u_vector_);
            x_vector_ = kkt_solver_.solve(rhs);

            // Relaxation for z-update
            VectorXd Ax = A_matrix_ * x_vector_;
            VectorXd Ax_hat = alpha_value_ * Ax + (1 - alpha_value_) * z_vector_;

            // z-update: projection onto bounds
            z_vector_ = Project(Ax_hat + u_vector_);

            // y-update: dual variable update
            u_vector_ +=  Ax_hat - z_vector_;

            // Compute residuals
            double r_norm = (Ax - z_vector_).norm();
            double s_norm = (rho_value_ * A_matrix_.transpose() * (z_vector_ - z_old)).norm();
            bool update_rho = false;
            if (r_norm > mu * s_norm && rho_value_ < rho_max_) {
                rho_value_ *= tau_incr;
                u_vector_ /= tau_incr;  // scale dual variable
                update_rho = true;
            } else if (s_norm > mu * r_norm && rho_value_ > rho_min_) {
                rho_value_ /= tau_decr;
                u_vector_ *= tau_decr;  // scale dual variable
                update_rho = true;
            }

            if (update_rho) {
                std::cout << "Iteration " << k+1 << ": rho updated to " << rho_value_ << std::endl;
                SetupKkt();  // recompute Cholesky factorization
            }

            // Dynamic rho update every check_interval iterations
            if (k == 0 || (k+1) % check_interval == 0) {
                // Compute tolerances
                double eps_pri = sqrt((double)m)*eps_abs_value_ + eps_rel_value_*std::max(Ax.norm(), z_vector_.norm());
                double eps_dual = sqrt((double)n)*eps_abs_value_ + eps_rel_value_*(A_matrix_.transpose()*u_vector_).norm();

                double obj_val = 0.5 * x_vector_.dot(P_matrix_ * x_vector_) + q_vector_.dot(x_vector_);
                std::cout << "Iter " << k+1 << ": obj_val = " << obj_val << ", r_norm = " << r_norm << ", s_norm = " << s_norm
                          << ", eps_pri = " << eps_pri << ", eps_dual = " << eps_dual << ", rho = " << rho_value_ << std::endl;

                // Check convergence
                if (r_norm <= eps_pri && s_norm <= eps_dual) {
                    std::cout << "Converged in " << k << " iterations." << std::endl;
                    break;
                }
            }

            z_old = z_vector_;  // save previous z for dual residual
        }
    }

    // Get the solution x
    VectorXd GetSolution() const {
        return x_vector_;
    }
};

int main() {
    int n = 2;
    int m = 2;

    // Define P matrix (quadratic term)
    SparseMatrix<double> P(n,n);
    std::vector<Eigen::Triplet<double>> tripletsP;
    tripletsP.emplace_back(0,0,4.0);
    tripletsP.emplace_back(1,1,2.0);
    P.setFromTriplets(tripletsP.begin(), tripletsP.end());

    // Define q vector (linear term)
    VectorXd q(n);
    q << 1.0, 1.0;

    // Define A matrix (constraints)
    SparseMatrix<double> A(m,n);
    std::vector<Eigen::Triplet<double>> tripletsA;
    tripletsA.emplace_back(0,0,1.0);
    tripletsA.emplace_back(1,1,1.0);
    A.setFromTriplets(tripletsA.begin(), tripletsA.end());

    // Define bounds
    VectorXd l(m), u(m);
    l << 0.0, 0.0;
    u << 1.0, 1.0;

    // Solve QP
    AdmmQpSolver solver(P, q, A, l, u);
    solver.Solve();

    std::cout << "Solution x = " << solver.GetSolution().transpose() << std::endl;
    return 0;
}

#endif //ADMM_QP_H
