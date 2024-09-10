#include "reference_line.h"

#include <algorithm>
#include <limits>
#include <unordered_set>
#include "boost/math/tools/minima.hpp"
#include "math/linear_interpolation.h"


namespace trajectory_utils {

    ReferenceLine::ReferenceLine(const std::vector<Vec2d> &reference_points) :
            reference_points_(reference_points) {
        num_points_ = static_cast<int>(reference_points_.size());
        accumulated_s_.clear();
        accumulated_s_.reserve(num_points_);
        segments_.clear();
        segments_.reserve(num_points_);
        double s = 0.0;
        for (int i = 0; i < num_points_; ++i) {
            accumulated_s_.push_back(s);

            Vec2d heading;
            if (i + 1 >= num_points_) {
                heading = reference_points_[i] - reference_points_[i - 1];
            } else {
                segments_.emplace_back(reference_points_[i], reference_points_[i + 1]);
                heading = reference_points_[i + 1] - reference_points_[i];
                s += heading.Length();
            }
        }

        num_segments_ = num_points_ - 1;
    }


    bool ReferenceLine::GetProjection(const common::math::Vec2d &point, double *accumulate_s,
                                      double *lateral) const {
        double min_distance = 0.0;

        if (segments_.empty()) {
            return false;
        }
        if (accumulate_s == nullptr || lateral == nullptr) {
            return false;
        }
        // CHECK_GE(num_points_, 2);
        min_distance = std::numeric_limits<double>::infinity();
        int min_index = 0;
        for (int i = 0; i < num_segments_; ++i) {
            const double distance = segments_[i].DistanceSquareTo(point);
            if (distance < min_distance) {
                min_index = i;
                min_distance = distance;
            }
        }
        min_distance = std::sqrt(min_distance);
        const auto &nearest_seg = segments_[min_index];
        const auto prod = nearest_seg.ProductOntoUnit(point);
        const auto proj = nearest_seg.ProjectOntoUnit(point);
        if (min_index == 0) {
            *accumulate_s = std::min(proj, nearest_seg.length());
            if (proj < 0) {
                *lateral = prod;
            } else {
                *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
            }
        } else if (min_index == num_segments_ - 1) {
            *accumulate_s = accumulated_s_[min_index] + std::max(0.0, proj);
            if (proj > 0) {
                *lateral = prod;
            } else {
                *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
            }
        } else {
            *accumulate_s = accumulated_s_[min_index] + std::max(0.0, std::min(proj, nearest_seg.length()));
            *lateral = (prod > 0.0 ? 1 : -1) * min_distance;
        }
        return true;
    }
}
