#pragma once

#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "proto/pnc_point.pb.h"
#include "math/vec2d.h"
#include "math/line_segment2d.h"

namespace trajectory_utils {

    using trajectory_utils::FrenetFramePoint;
    using trajectory_utils::PathPoint;
    using trajectory_utils::SLPoint;
    using trajectory_utils::TrajectoryPoint;
    using trajectory_utils::common::math::Vec2d;

    class ReferenceLine {
    public:
        ReferenceLine() = default;

        ReferenceLine(const std::vector<Vec2d> &reference_points);

        bool GetProjection(const common::math::Vec2d &point, double *accumulate_s, double *lateral) const;

        std::vector<Vec2d> reference_points_;
        std::vector<double> accumulated_s_;

        int num_points_ = 0;
        int num_segments_ = 0;
        std::vector<common::math::LineSegment2d> segments_;

        int number_ = std::numeric_limits<int>::max();
    };

}
