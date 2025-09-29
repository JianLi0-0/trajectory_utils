//
// Created by desmond on 2025/9/28.
//

#ifndef ADMM_QP_H
#define ADMM_QP_H

#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <iostream>
#include <vector>

using Eigen::VectorXd;
using Eigen::SparseMatrix;

// ADMM-based QP solver with augmented KKT solve (OSQP-style)
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

    // Augmented KKT ( (n+m)x(n+m) )
    SparseMatrix<double> K_aug_;
    Eigen::SparseLU<SparseMatrix<double>> kkt_aug_solver_;
    bool kkt_aug_ready_ = false;

    AdmmQpSolver(const SparseMatrix<double>& P, const VectorXd& q,
                 const SparseMatrix<double>& A, const VectorXd& l, const VectorXd& u)
        : P_matrix_(P), q_vector_(q), A_matrix_(A), lA_vector_(l), uA_vector_(u) {
        int n = P.rows();
        int m = A.rows();
        x_vector_ = VectorXd::Zero(n);
        z_vector_ = VectorXd::Zero(m);
        u_vector_ = VectorXd::Zero(m);
        GetStartPoint();
        SetupAugKkt();
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

    // Build augmented KKT:
    // [ P + σ I      A^T ]
    // [   A       - (1/ρ) I ]
    void SetupAugKkt() {
        int n = P_matrix_.rows();
        int m = A_matrix_.rows();
        std::vector<Eigen::Triplet<double>> triplets;
        triplets.reserve(P_matrix_.nonZeros() + A_matrix_.nonZeros()*2 + n + m);

        // Top-left: P + σ I
        for (int k=0; k<P_matrix_.outerSize(); ++k)
            for (SparseMatrix<double>::InnerIterator it(P_matrix_, k); it; ++it)
                triplets.emplace_back(it.row(), it.col(), it.value());
        for (int i=0;i<n;++i)
            triplets.emplace_back(i,i, sigma_value_);

        // Top-right: A^T
        Eigen::SparseMatrix<double> At = A_matrix_.transpose();
        for (int k=0; k<At.outerSize(); ++k)
            for (SparseMatrix<double>::InnerIterator it(At, k); it; ++it)
                triplets.emplace_back(it.row(), n + it.col(), it.value());

        // Bottom-left: A
        for (int k=0; k<A_matrix_.outerSize(); ++k)
            for (SparseMatrix<double>::InnerIterator it(A_matrix_, k); it; ++it)
                triplets.emplace_back(n + it.row(), it.col(), it.value());

        // Bottom-right: - (1/ρ) I_m
        double neg_inv_rho = -1.0 / rho_value_;
        for (int i=0;i<m;++i)
            triplets.emplace_back(n + i, n + i, neg_inv_rho);

        K_aug_.resize(n + m, n + m);
        K_aug_.setFromTriplets(triplets.begin(), triplets.end());

        kkt_aug_solver_.analyzePattern(K_aug_);
        kkt_aug_solver_.factorize(K_aug_);
        if (kkt_aug_solver_.info() != Eigen::Success)
            throw std::runtime_error("Augmented KKT factorization failed.");
        kkt_aug_ready_ = true;
    }

    // Solve augmented system to obtain x_tilde, ν
    // RHS:
    // top: σ x^k - q
    // bottom: z^k - u^k   (u = y/ρ)
    void UpdateXZTilde(VectorXd& x_tilde, VectorXd& nu) {
        if (!kkt_aug_ready_) SetupAugKkt();
        int n = P_matrix_.rows();
        int m = A_matrix_.rows();

        VectorXd rhs(n + m);
        rhs.head(n) = sigma_value_ * x_vector_ - q_vector_;
        rhs.tail(m) = z_vector_ - u_vector_;

        VectorXd sol = kkt_aug_solver_.solve(rhs);
        if (kkt_aug_solver_.info() != Eigen::Success)
            throw std::runtime_error("Augmented KKT solve failed.");

        x_tilde = sol.head(n);
        nu      = sol.tail(m);
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

        double mu = 10.0;
        double tau_incr = 2.0;
        double tau_decr = 2.0;
        int check_interval = 25;

        for (int k = 0; k < max_iter_value_; ++k) {
            // 1. Solve augmented KKT for x_tilde, z_tilde
            VectorXd x_tilde, z_tilde, v;
            UpdateXZTilde(x_tilde, v);
            z_tilde = z_vector_ + v / rho_value_ - u_vector_;

            // 2. x update
            x_vector_ = alpha_value_ * x_tilde + (1.0 - alpha_value_) * x_vector_;

            // 3. z-update (projection), z_tilde = A * x_tilde
            VectorXd z_relaxation = alpha_value_*z_tilde + (1-alpha_value_)*z_vector_;
            z_vector_ = Project( z_relaxation + u_vector_);

            // 4. Dual (scaled) update
            u_vector_ += z_relaxation - z_vector_;

            // 6. Residuals
            VectorXd Ax = A_matrix_ * x_vector_;
            double r_norm = (Ax - z_vector_).norm();
            double s_norm = (rho_value_ * A_matrix_.transpose() * (z_vector_ - z_old)).norm();
            z_old = z_vector_;

            bool rho_changed = false;
            if (r_norm > mu * s_norm && rho_value_ < rho_max_) {
                rho_value_ *= tau_incr;
                u_vector_ /= tau_incr;
                rho_changed = true;
            } else if (s_norm > mu * r_norm && rho_value_ > rho_min_) {
                rho_value_ /= tau_decr;
                u_vector_ *= tau_decr;
                rho_changed = true;
            }
            if (rho_changed) {
                SetupAugKkt(); // ρ appears in -1/ρ I
                std::cout << "Iter " << k+1 << ": rho -> " << rho_value_ << std::endl;
            }

            // Dynamic rho update every check_interval iterations
            if (k == 0 || (k+1) % check_interval == 0) {
                double eps_pri = std::sqrt((double)m)*eps_abs_value_ +
                                 eps_rel_value_*std::max(Ax.norm(), z_vector_.norm());
                double eps_dual = std::sqrt((double)n)*eps_abs_value_ +
                                  eps_rel_value_*(A_matrix_.transpose()*u_vector_).norm();
                double obj_val = 0.5 * x_vector_.dot(P_matrix_ * x_vector_) + q_vector_.dot(x_vector_);

                std::cout << "Iter " << k+1
                          << ": obj=" << obj_val
                          << " r=" << r_norm
                          << " s=" << s_norm
                          << " eps_pri=" << eps_pri
                          << " eps_dual=" << eps_dual
                          << " rho=" << rho_value_ << std::endl;

                if (r_norm <= eps_pri && s_norm <= eps_dual) {
                    std::cout << "Converged in " << k << " iterations." << std::endl;
                    break;
                }
            }
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
