#include "trajectory_info.h"

namespace trajectory_utils {

    TrajectoryInfo::TrajectoryInfo() {
        trajectory_ptr = std::shared_ptr<DiscretizedTrajectory>(
                new DiscretizedTrajectory());

        ruckig_input_.current_position = {0.0};
        ruckig_input_.current_velocity = {0.0};
        ruckig_input_.current_acceleration = {0.0};

        ruckig_input_.target_position = {0.0};
        ruckig_input_.target_velocity = {0.0};
        ruckig_input_.target_acceleration = {0.0};

        ruckig_input_.max_velocity = {1.75};
        ruckig_input_.max_acceleration = {2.0};
        ruckig_input_.max_jerk = {4.0};

        // Set different constraints for negative direction
        ruckig_input_.min_velocity = {-1e-3};
        ruckig_input_.min_acceleration = {-2.0};
    }

    void TrajectoryInfo::reset() {
        ROS_INFO("Reset trajectory info.");
        reference_line_ptr_.reset();
        trajectory_ptr.reset();
        path_data_.clear();
        speed_data_ = ruckig::Trajectory<1>();
        ruckig_input_.current_position = {0.0};
        ruckig_input_.current_velocity = {0.0};
        ruckig_input_.current_acceleration = {0.0};
    }

    bool TrajectoryInfo::setPathData(const std::vector<PathPoint> &path_data) {
        if (!DiscretePointsMath::computePathInfo(path_data, path_data_)) {
            ROS_ERROR("Fail to compute path info.");
            return false;
        }

        std::vector<Vec2d> reference_points;
        for (const auto &path_point : path_data_) {
            reference_points.emplace_back(path_point.x(), path_point.y());
        }

        reference_line_ptr_ = std::shared_ptr<ReferenceLine>(new ReferenceLine(reference_points));

        return true;
    }

    bool TrajectoryInfo::calSpeedData(const double& cur_pos, const double& cur_speed,
                                      const double& cur_acc, const double& tar_pos) {
        ruckig_input_.current_position[0] = cur_pos;
        ruckig_input_.current_velocity[0] = cur_speed;
        ruckig_input_.current_acceleration[0] = cur_acc;
        ruckig_input_.target_position[0] = tar_pos;

        auto result = ruckig_otg_.calculate(ruckig_input_, speed_data_);

        if (result != ruckig::Result::Working && result != ruckig::Result::Finished) {
            ROS_ERROR("Speed planning calculation error. Error code: %d", result);
            return false;
        }

        return true;
    }

    bool TrajectoryInfo::combinePathAndSpeedProfile() {
        const double timeResoltuion = 0.02;

        if (path_data_.empty()) {
            ROS_ERROR("path data is empty");
            return false;
        }

        trajectory_ptr.reset();
        trajectory_ptr = std::shared_ptr<DiscretizedTrajectory>(
                new DiscretizedTrajectory());

        for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.get_duration();
             cur_rel_time += timeResoltuion) {
            SpeedPoint speed_point;

            std::array<double, 1> new_position, new_velocity, new_acceleration;
            speed_data_.at_time(cur_rel_time, new_position, new_velocity, new_acceleration);

            speed_point.set_s(new_position[0]);
            speed_point.set_v(new_velocity[0]);
            speed_point.set_a(new_acceleration[0]);
            speed_point.set_t(cur_rel_time);

            if (speed_point.s() > path_data_.Length()) {
                break;
            }
            PathPoint path_point = path_data_.Evaluate(speed_point.s());
            path_point.set_s(path_point.s());

            TrajectoryPoint trajectory_point;
            trajectory_point.mutable_path_point()->CopyFrom(path_point);
            trajectory_point.set_v(speed_point.v());
            trajectory_point.set_a(speed_point.a());
            trajectory_point.set_relative_time(speed_point.t());
            trajectory_ptr->AppendTrajectoryPoint(trajectory_point);
        }
        return true;
    }

    bool TrajectoryInfo::getRefTrajectoryPoint(const Vec2d& position, TrajectoryPoint& ref_point) {
        if (!reference_line_ptr_) {
            ROS_WARN("Reference line is not set.");
            return false;
        }

        if (!trajectory_ptr) {
            ROS_WARN("Trajectory is not set.");
            return false;
        }

        double s, l, t;
        reference_line_ptr_->GetProjection(position, &s, &l);
        std::cout << "s: " << s << ", l: " << l << std::endl;
        speed_data_.get_first_time_at_position(0, s, t);
        ref_point = trajectory_ptr->Evaluate(t);
        return true;
    }
} // trajectory_utils