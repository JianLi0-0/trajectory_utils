#ifndef MPC_ACC_H_
#define MPC_ACC_H_

#include "mpc.h"
#include <Eigen/Eigen>
#include <vector>
#include <Eigen/SparseLU>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/tf.h"
#include "trajectory_info.h"

class MPC_ACC : public MPC {
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

    // 重写calculateVelocity方法，处理加速度积分
    bool calculateVelocity(const geometry_msgs::PoseStamped& current_pose, 
                          geometry_msgs::Twist& cmd_vel);

    // 初始化方法，添加加速度约束
    void init(double v_max, double w_max, double omega0, double omega1_v, double omega1_w,
              double a_max = 2.0) {
        MPC::init(v_max, w_max, omega0, omega1_v, omega1_w);
        a_max_ = a_max;
        a_min_ = -a_max;
        
        // 初始化当前速度
        current_v_ = 0.0;
    }

    // 设置当前速度状态
    void setCurrentVelocity(double v) {
        current_v_ = v;
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
    
    double current_v_;  // 当前线速度
    
    std::vector<geometry_msgs::PoseStamped> mpc_traj_;  // MPC计算的预测轨迹
    
    const double t_step_acc_ = 0.05;  // 时间步长
    const int N_acc_ = 20;            // 预测步数
};

#endif