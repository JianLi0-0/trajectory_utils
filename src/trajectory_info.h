#ifndef SRC_TRAJECTORY_INFO_H
#define SRC_TRAJECTORY_INFO_H

#include "discretized_trajectory.h"
#include "discretized_path.h"
#include "math/discrete_points_math.h"
#include "reference_line.h"
#include <ros/ros.h>
#include <ruckig/ruckig.hpp>

namespace trajectory_utils {

    class TrajectoryInfo {
        public:

        TrajectoryInfo();
        ~TrajectoryInfo() = default;

        void reset();
        bool combinePathAndSpeedProfile();
        bool setPathData(const std::vector<PathPoint> &path_data);
        void setSpeedData(const ruckig::Trajectory<1> &speed_data) { speed_data_ = speed_data; }
        bool calSpeedData(const double& cur_pos, const double& cur_speed,
                          const double& cur_acc, const double& tar_pos);

        std::shared_ptr<DiscretizedTrajectory> getTrajectoryPtr() { return trajectory_ptr; }
        DiscretizedPath* getPathDataPtr() { return &path_data_; }
        ruckig::Trajectory<1>* getSpeedDataPtr() { return &speed_data_; }
        ruckig::InputParameter<1>* getSpeedPlanningParamsPtr() { return &ruckig_input_; }
        bool getRefTrajectoryPoint(const Vec2d& position, TrajectoryPoint& ref_point);

        private:
        std::shared_ptr<ReferenceLine> reference_line_ptr_;
        std::shared_ptr<DiscretizedTrajectory> trajectory_ptr;
        DiscretizedPath path_data_;
        ruckig::Trajectory<1> speed_data_;
        ruckig::InputParameter<1> ruckig_input_;
        ruckig::Ruckig<1> ruckig_otg_;

    };

} // trajectory_utils

#endif //SRC_TRAJECTORY_INFO_H
