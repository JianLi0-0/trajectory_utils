#include "trajectory_info.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace trajectory_utils {

    TrajectoryInfo::TrajectoryInfo() {

        ruckig_input_.current_position = {0.0};
        ruckig_input_.current_velocity = {0.0};
        ruckig_input_.current_acceleration = {0.0};

        ruckig_input_.target_position = {0.0};
        ruckig_input_.target_velocity = {0.0};
        ruckig_input_.target_acceleration = {0.0};

//        ruckig_input_.max_velocity = {1.75};
//        ruckig_input_.max_acceleration = {0.5};
//        ruckig_input_.max_jerk = {1.0};

        // Set different constraints for negative direction
        ruckig_input_.min_velocity = {-1e-3};
//        ruckig_input_.min_acceleration = {-0.5};
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
                                      const double& cur_acc, const double& tar_pos,
                                      const double& max_speed, const double& max_jerk,
                                      const double& max_acc, const double& min_acc) {
        ruckig_input_.current_position[0] = cur_pos;
        ruckig_input_.current_velocity[0] = cur_speed;
        ruckig_input_.current_acceleration[0] = cur_acc;
        ruckig_input_.target_position[0] = tar_pos;
        ruckig_input_.max_velocity[0] = max_speed;

        ruckig_input_.max_jerk = {max_jerk};
        ruckig_input_.max_acceleration = {max_acc};
        ruckig_input_.min_acceleration = {min_acc};

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

    bool TrajectoryInfo::findKappaMax(const double& max_s, double& max_kappa) {
        max_kappa = 0.0;
        for (auto p:path_data_) {
            if (p.s() > max_s) {
                break;
            }
            double kappa_abs = std::fabs(p.kappa());
            if (kappa_abs > max_kappa) {
                max_kappa = kappa_abs;
            }
        }
        return true;
    }

    void TrajectoryInfo::displayTrajProfile() {

        if (!trajectory_ptr) {
            ROS_WARN("Trajectory is not set.");
            return;
        }

        std::vector<double> t_vec;
        double time_span = getSpeedDataPtr()->get_duration();
        for (double t=0.0;t<=time_span;t+=0.01) {
            t_vec.push_back(t);
        }

        std::vector<double> x_traj_vec, y_traj_vec, s_traj_vec, v_traj_vec, a_traj_vec, kappa_traj_vec;
        x_traj_vec.reserve(t_vec.size());
        y_traj_vec.reserve(t_vec.size());
        s_traj_vec.reserve(t_vec.size());
        v_traj_vec.reserve(t_vec.size());
        a_traj_vec.reserve(t_vec.size());
        kappa_traj_vec.reserve(t_vec.size());
        for (auto t : t_vec) {
            auto traj_point = trajectory_ptr->Evaluate(t);
            x_traj_vec.push_back(traj_point.path_point().x());
            y_traj_vec.push_back(traj_point.path_point().y());
            s_traj_vec.push_back(traj_point.path_point().s());
            kappa_traj_vec.push_back(traj_point.path_point().kappa());
            v_traj_vec.push_back(traj_point.v());
            a_traj_vec.push_back(traj_point.a());
        }

        plt::figure_size(640, 640);
        plt::plot(t_vec, s_traj_vec);
        plt::plot(t_vec, v_traj_vec);
        plt::plot(t_vec, a_traj_vec);
        plt::plot(t_vec, kappa_traj_vec);
//        plt::plot(x_traj_vec, y_traj_vec);
        plt::title("Traj Profile");
        plt::grid(true);

        plt::show();

    }
} // trajectory_utils