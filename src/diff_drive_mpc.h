#ifndef DIFF_DRIVE_MPC_H
#define DIFF_DRIVE_MPC_H

#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <memory>

/**
 * @brief N步线性化差速轮MPC类
 * 控制输入: a (线加速度), omega (角速度)
 * 状态: x, y, theta, v
 * 支持:
 * - 每步朝向跟踪软约束
 * - 终端距离软约束
 * - 控制输入约束
 */
class DiffDriveMPC {
public:
    DiffDriveMPC(int N, double Ts, costmap_2d::Costmap2D* costmap_ptr);

    void setConstraints(double a_min, double a_max, double w_min, double w_max, double v_max);

    void setWeights(double w_a, double w_omega, double w_r, double w_theta);

    bool solve(const Eigen::Vector4d &state, const Eigen::Vector2d &target, double d_des, Eigen::Vector2d &u_opt);

    const std::vector<geometry_msgs::PoseStamped>& getTrajectory() const { return mpc_traj_; }

    const std::vector<geometry_msgs::PoseStamped>& getReferenceTrajectory() const { return reference_traj_; }

private:
    void generateDistanceMap();

    int N_, n_state_, n_control_;
    double Ts_;
    double w_a_, w_omega_;
    double w_position_, w_theta_;
    double a_min_, a_max_, w_min_, w_max_, v_max_;
    Eigen::Vector2d last_u_;
    std::vector<geometry_msgs::PoseStamped> mpc_traj_;
    std::vector<geometry_msgs::PoseStamped> reference_traj_;
    costmap_2d::Costmap2D* costmap_ptr_;
    grid_map::GridMap map_;
    std::unique_ptr<grid_map::SignedDistanceField> sdf_;
};

#endif //DIFF_DRIVE_MPC_H
