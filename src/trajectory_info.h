#ifndef SRC_TRAJECTORY_INFO_H
#define SRC_TRAJECTORY_INFO_H

#include "discretized_trajectory.h"
#include "discretized_path.h"
#include "math/discrete_points_math.h"
#include "reference_line.h"
#include <ros/ros.h>
#include <ruckig/ruckig.hpp>
#include <geometry_msgs/PoseStamped.h>

namespace trajectory_utils {

    struct CurvatureSpeedLimit {
        double cruise_speed = 1.0;
        double avoid_obs_vel_min = 0.5;
        double avoid_obs_vel_max = cruise_speed;
        double avoid_obs_kappa_min = 0.3;
        double avoid_obs_kappa_max = 1.0;
    };

    class TrajectoryInfo {
        public:

        TrajectoryInfo();
        ~TrajectoryInfo() = default;

        void reset();
        bool combinePathAndSpeedProfile();
        bool setPathData(const std::vector<PathPoint> &path_data);
        void setSpeedData(const ruckig::Trajectory<1> &speed_data) { speed_data_ = speed_data; }
        bool calSpeedData(const double& cur_pos, const double& cur_speed,
                          const double& cur_acc, const double& tar_pos, const double& max_speed=1.75);

        std::shared_ptr<DiscretizedTrajectory> getTrajectoryPtr() { return trajectory_ptr; }
        DiscretizedPath* getPathDataPtr() { return &path_data_; }
        std::shared_ptr<ReferenceLine> getReferenceLinePtr() { return reference_line_ptr_; }
        ruckig::Trajectory<1>* getSpeedDataPtr() { return &speed_data_; }
        ruckig::InputParameter<1>* getSpeedPlanningParamsPtr() { return &ruckig_input_; }
        bool getRefTrajectoryPoint(const Vec2d& position, TrajectoryPoint& ref_point);
        bool findKappaMax(const double& max_s, double& max_kappa);
        double getRefKappa(const Vec2d& position);
        void displayTrajProfile();
        bool longitudinalSpeedPlanning(
                const std::vector<geometry_msgs::PoseStamped>& path, const double& last_vel, double& output_vel, double max_speed=1.0, double time_interval=0.1);
        void setCurvatureSpeedLimit(const double& cruise_speed, const double& avoid_obs_vel_min,
                                    const double& avoid_obs_vel_max, const double& avoid_obs_kappa_max) {
            curvature_speed_limit_.cruise_speed = cruise_speed;
            curvature_speed_limit_.avoid_obs_vel_min = avoid_obs_vel_min;
            curvature_speed_limit_.avoid_obs_vel_max = avoid_obs_vel_max;
            curvature_speed_limit_.avoid_obs_kappa_max = avoid_obs_kappa_max;
        }
        void setAccLimit(const double& max_acc, const double& min_acc) {
            ruckig_input_.max_acceleration = {max_acc};
            ruckig_input_.min_acceleration = {min_acc};
        }
        CurvatureSpeedLimit& getCurvatureSpeedLimit() { return curvature_speed_limit_; }
        void displayUpdate(const double& x, const double& y);

        private:
        std::shared_ptr<ReferenceLine> reference_line_ptr_;
        std::shared_ptr<DiscretizedTrajectory> trajectory_ptr;
        DiscretizedPath path_data_;
        ruckig::Trajectory<1> speed_data_;
        ruckig::InputParameter<1> ruckig_input_;
        ruckig::Ruckig<1> ruckig_otg_;
        CurvatureSpeedLimit curvature_speed_limit_;
        trajectory_utils::TrajectoryPoint traj_point_;

    };

} // trajectory_utils

#endif //SRC_TRAJECTORY_INFO_H
