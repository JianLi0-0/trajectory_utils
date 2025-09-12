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
     * 使用线性误差模型，用qpOASES求解
     * @param X_c 当前状态
     * @param X_r 参考状态
     * @param U_r 参考收入
     * @param N   预测步数
     * @return    预测输出序列
     */
    Eigen::MatrixXd solve(Eigen::Vector3d X_k, std::vector<Eigen::Vector3d> X_r,
                          std::vector<Eigen::Vector2d> U_r, const int N);

    void init(double v_max, double w_max, double omega0, double omega1_v, double omega1_w) {
        v_max_ = v_max;
        v_min_ = 0.0;
        w_max_ = w_max;
        w_min_ = -w_max;
        omega0_ = omega0;
        omega1_v_ = omega1_v;
        omega1_w_ = omega1_w;

        ref_v_max_ = v_max;
    }

    void setRefVMax(double v_max) {ref_v_max_ = v_max;}

    void setReferenceSpeedProfileParam(double max_acc, double min_acc) {
        trajectory_info_.setAccLimit(max_acc, min_acc);
    }

    void generateReferenceTrajectory(const geometry_msgs::PoseStamped& current_pose,
                                     const std::vector<geometry_msgs::PoseStamped>& path, const double& initial_linear_vel=0.0);

    bool calculateVelocity(const geometry_msgs::PoseStamped& current_pose, geometry_msgs::Twist& cmd_vel);

    void setSaveDistance(const double& save_distance) {
        save_distance_ = save_distance;
    }

    void reset() {
        trajectory_info_.reset();
    };

    trajectory_utils::TrajectoryInfo& getTrajectoryInfo() {
        return trajectory_info_;
    }

protected:
    double v_max_;
    double v_min_;
    double w_max_;
    double w_min_;
    double omega0_, omega1_v_, omega1_w_;
    double ref_v_max_;

    const double t_step = 0.03;
    const int N = 20;
    double save_distance_ = 1.2;
    double traj_duration_;
    trajectory_utils::TrajectoryInfo trajectory_info_;
};

#endif