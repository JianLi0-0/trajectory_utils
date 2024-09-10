#pragma once

#include <utility>
#include <vector>
#include "../proto/pnc_point.pb.h"

namespace trajectory_utils {

    class DiscretePointsMath {
    public:
        DiscretePointsMath() = delete;

        static bool computePathProfile(const std::vector<std::pair<double, double>> &xy_points,
                                       std::vector<double> *headings, std::vector<double> *accumulated_s,
                                       std::vector<double> *kappas, std::vector<double> *dkappas);

        static bool computePathInfo(const std::vector<PathPoint>& raw_path_points,
                                    std::vector<PathPoint> &path_points);
    };

}

