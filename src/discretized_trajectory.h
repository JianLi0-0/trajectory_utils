#pragma once

#include <vector>

#include "math/vec2d.h"
#include "proto/pnc_point.pb.h"

namespace trajectory_utils {


    class DiscretizedTrajectory : public std::vector<TrajectoryPoint> {
    public:
        DiscretizedTrajectory() = default;

        explicit DiscretizedTrajectory(
                const std::vector<TrajectoryPoint> &trajectory_points);

        void SetTrajectoryPoints(
                const std::vector<TrajectoryPoint> &trajectory_points);

        virtual ~DiscretizedTrajectory() = default;

        virtual TrajectoryPoint StartPoint() const;

        virtual double GetTemporalLength() const;

        virtual double GetSpatialLength() const;

        virtual TrajectoryPoint Evaluate(const double relative_time) const;

        virtual size_t QueryLowerBoundPoint(const double relative_time,
                                            const double epsilon = 1.0e-5) const;

        virtual size_t QueryNearestPoint(const common::math::Vec2d &position) const;

        size_t QueryNearestPointWithBuffer(const common::math::Vec2d &position,
                                           const double buffer) const;

        virtual void AppendTrajectoryPoint(
                const TrajectoryPoint &trajectory_point);

        void PrependTrajectoryPoints(
                const std::vector<TrajectoryPoint> &trajectory_points) {
            if (!empty() && trajectory_points.size() > 1) {
                //   ACHECK(trajectory_points.back().relative_time() <
                //          front().relative_time());
            }
            insert(begin(), trajectory_points.begin(), trajectory_points.end());
        }

        const TrajectoryPoint &TrajectoryPointAt(const size_t index) const;

        size_t NumOfPoints() const;

        virtual void Clear();
    };

    inline size_t DiscretizedTrajectory::NumOfPoints() const { return size(); }

    inline void DiscretizedTrajectory::Clear() { clear(); }

}