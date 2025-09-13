#ifndef MPC_H_
#define MPC_H_

#include <Eigen/Eigen>
#include <vector>
#include <Eigen/SparseLU>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "trajectory_info.h"

class MPC {
public:
    /****
     * 使用线性误差模型，用OSQP求解线加速度与角速度
     * @param X_k 当前状态 [x, y, theta, v] - 4维状态
     * @param X_r 参考状态 [x, y, theta, v] - 4维参考状态
     * @param U_r 参考输入 [a_ref, w_ref] - 加速度参考
     * @param N   预测步数
     * @return    预测输出序列 [a, w] - 线加速度与角速度(无角加速度)
     */
    Eigen::MatrixXd solve(Eigen::Vector4d X_k,
                          std::vector<Eigen::Vector4d> X_r,
                          std::vector<Eigen::Vector2d> U_r, const int N);

    // 初始化方法，添加加速度约束
    void init(double v_max, double w_max, double omega0, double omega1_v, double omega1_w,
              double a_max = 2.0) {
        v_max_ = v_max;
        v_min_ = 0.0;
        w_max_ = w_max;
        w_min_ = -w_max;
        omega0_ = omega0;
        omega1_v_ = omega1_v;
        omega1_w_ = omega1_w;
        a_max_ = a_max;
        a_min_ = -a_max;
    }

    void setReferenceSpeedProfileParam(double max_acc, double min_acc) {
        trajectory_info_.setAccLimit(max_acc, min_acc);
    }

    void generateReferenceTrajectory(const geometry_msgs::PoseStamped& current_pose,
                                     const std::vector<geometry_msgs::PoseStamped>& path, const double& initial_linear_vel=0.0);

    bool calculateVelocity(const geometry_msgs::PoseStamped& current_pose,
        geometry_msgs::Twist& cmd_vel, const double& current_v);

    void setSaveDistance(const double& save_distance) {
        save_distance_ = save_distance;
    }

    void reset() {
        trajectory_info_.reset();
    };

    trajectory_utils::TrajectoryInfo& getTrajectoryInfo() {
        return trajectory_info_;
    }

    // 获取MPC计算的预测轨迹
    const std::vector<geometry_msgs::PoseStamped>& getMpcTrajectory() const {
        return mpc_traj_;
    }

private:
    // 计算MPC预测轨迹
    void calculateMpcTrajectory(const Eigen::Vector4d& X_k, const Eigen::MatrixXd& u_k);

    double a_max_;      // 最大线加速度
    double a_min_;      // 最小线加速度
    double v_max_;
    double v_min_;
    double w_max_;
    double w_min_;
    double omega0_, omega1_v_, omega1_w_;

    const double t_step_ = 0.05;  // 时间步长
    const int horizon_ = 10;            // 预测步数
    double save_distance_ = 1.2;
    double traj_duration_;
    trajectory_utils::TrajectoryInfo trajectory_info_;
    
    std::vector<geometry_msgs::PoseStamped> mpc_traj_;  // MPC计算的预测轨迹

    std::vector<double> kappa_ref_vec_;
};

#endif