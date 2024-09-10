#include <limits>

#include "discretized_trajectory.h"
#include "math/linear_interpolation.h"

#include <ros/ros.h>

namespace trajectory_utils {

    DiscretizedTrajectory::DiscretizedTrajectory(
            const std::vector<TrajectoryPoint> &trajectory_points)
            : std::vector<TrajectoryPoint>(trajectory_points) {
//   ACHECK(!trajectory_points.empty())
//       << "trajectory_points should NOT be empty()";
        if (trajectory_points.empty())
            ROS_ERROR("trajectory_points should NOT be empty()");
    }

    TrajectoryPoint DiscretizedTrajectory::Evaluate(
            const double relative_time) const {
        auto comp = [](const TrajectoryPoint &p, const double relative_time) {
            return p.relative_time() < relative_time;
        };

        auto it_lower = std::lower_bound(begin(), end(), relative_time, comp);

        if (it_lower == begin()) {
            return front();
        } else if (it_lower == end()) {
            ROS_WARN("When evaluate trajectory, relative_time(%f) is too large", relative_time);
            return back();
        }
        return common::math::InterpolateUsingLinearApproximation(
                *(it_lower - 1), *it_lower, relative_time);
    }

    size_t DiscretizedTrajectory::QueryLowerBoundPoint(const double relative_time,
                                                       const double epsilon) const {
        // ACHECK(!empty());
        if (empty()) ROS_ERROR("empty()");
        if (relative_time >= back().relative_time()) {
            return size() - 1;
        }
        auto func = [&epsilon](const TrajectoryPoint &tp,
                               const double relative_time) {
            return tp.relative_time() + epsilon < relative_time;
        };
        auto it_lower = std::lower_bound(begin(), end(), relative_time, func);
        return std::distance(begin(), it_lower);
    }

    size_t DiscretizedTrajectory::QueryNearestPoint(
            const common::math::Vec2d &position) const {
        double dist_sqr_min = std::numeric_limits<double>::max();
        size_t index_min = 0;
        for (size_t i = 0; i < size(); ++i) {
            const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                                 data()[i].path_point().y());

            const double dist_sqr = curr_point.DistanceSquareTo(position);
            if (dist_sqr < dist_sqr_min) {
                dist_sqr_min = dist_sqr;
                index_min = i;
            }
        }
        return index_min;
    }

    size_t DiscretizedTrajectory::QueryNearestPointWithBuffer(
            const common::math::Vec2d &position, const double buffer) const {
        double dist_sqr_min = std::numeric_limits<double>::max();
        size_t index_min = 0;
        for (size_t i = 0; i < size(); ++i) {
            const common::math::Vec2d curr_point(data()[i].path_point().x(),
                                                 data()[i].path_point().y());

            const double dist_sqr = curr_point.DistanceSquareTo(position);
            if (dist_sqr < dist_sqr_min + buffer) {
                dist_sqr_min = dist_sqr;
                index_min = i;
            }
        }
        return index_min;
    }

    void DiscretizedTrajectory::AppendTrajectoryPoint(
            const TrajectoryPoint &trajectory_point) {
        if (!empty()) {
            ROS_ERROR_COND(trajectory_point.relative_time() < back().relative_time(),
                           "The appended point is not in chronological order.");
        }
        push_back(trajectory_point);
    }

    const TrajectoryPoint &DiscretizedTrajectory::TrajectoryPointAt(
            const size_t index) const {
        ROS_ERROR_COND(index >= NumOfPoints(), "index is out of bound.");
        return data()[index];
    }

    TrajectoryPoint DiscretizedTrajectory::StartPoint() const {
        // ACHECK(!empty());
        if (empty()) ROS_ERROR("empty()");
        return front();
    }

    double DiscretizedTrajectory::GetTemporalLength() const {
        if (empty()) {
            return 0.0;
        }
        return back().relative_time() - front().relative_time();
    }

    double DiscretizedTrajectory::GetSpatialLength() const {
        if (empty()) {
            return 0.0;
        }
        return back().path_point().s() - front().path_point().s();
    }

}