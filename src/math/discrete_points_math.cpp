#include "discrete_points_math.h"
#include <cmath>

namespace trajectory_utils {

    bool DiscretePointsMath::computePathInfo(const std::vector<PathPoint>& raw_path_points,
                                             std::vector<PathPoint> &path_points) {
        if (raw_path_points.empty()) {
            return false;
        }

        std::vector<std::pair<double, double>> smoothed_point2d;
        std::pair<double, double> point2d;
        for (size_t i = 0; i < raw_path_points.size(); i++) {
            point2d.first = raw_path_points.at(i).x();
            point2d.second = raw_path_points.at(i).y();
            smoothed_point2d.push_back(point2d);
        }
        // 求解曲率等
        std::vector<double> headings;
        std::vector<double> kappas;
        std::vector<double> dkappas;
        std::vector<double> accumulated_s;
        if (!computePathProfile(smoothed_point2d, &headings,
                                &accumulated_s, &kappas, &dkappas)) {
            return false;
        }

        PathPoint p;
        path_points.clear();
        path_points.reserve(smoothed_point2d.size());
        for (size_t i = 0; i < smoothed_point2d.size(); i++) {
            p.set_x(smoothed_point2d[i].first);
            p.set_y(smoothed_point2d[i].second);
            p.set_theta(headings[i]);
            p.set_s(accumulated_s[i]);
            p.set_kappa(kappas[i]);
            p.set_dkappa(dkappas[i]);
            path_points.push_back(p);
        }

        return true;
    }

    bool DiscretePointsMath::computePathProfile(const std::vector<std::pair<double, double>> &xy_points,
                                                std::vector<double> *headings, std::vector<double> *accumulated_s,
                                                std::vector<double> *kappas, std::vector<double> *dkappas) {
        headings->clear();
        kappas->clear();
        dkappas->clear();

        if (xy_points.size() < 2) {
            return false;
        }
        std::vector<double> dxs;
        std::vector<double> dys;
        std::vector<double> y_over_s_first_derivatives;
        std::vector<double> x_over_s_first_derivatives;
        std::vector<double> y_over_s_second_derivatives;
        std::vector<double> x_over_s_second_derivatives;

        // Get finite difference approximated dx and dy for heading and kappa
        // calculation
        std::size_t points_size = xy_points.size();
        for (std::size_t i = 0; i < points_size; ++i) {
            double x_delta = 0.0;
            double y_delta = 0.0;
            if (i == 0) {
                x_delta = (xy_points[i + 1].first - xy_points[i].first);
                y_delta = (xy_points[i + 1].second - xy_points[i].second);
            } else if (i == points_size - 1) {
                x_delta = (xy_points[i].first - xy_points[i - 1].first);
                y_delta = (xy_points[i].second - xy_points[i - 1].second);
            } else {
                x_delta = 0.5 * (xy_points[i + 1].first - xy_points[i - 1].first);
                y_delta = 0.5 * (xy_points[i + 1].second - xy_points[i - 1].second);
            }
            dxs.push_back(x_delta);
            dys.push_back(y_delta);
        }

        // Heading calculation
        for (std::size_t i = 0; i < points_size; ++i) {
            headings->push_back(std::atan2(dys[i], dxs[i]));
        }

        // Get linear interpolated s for dkappa calculation
        double distance = 0.0;
        accumulated_s->push_back(distance);
        double fx = xy_points[0].first;
        double fy = xy_points[0].second;
        double nx = 0.0;
        double ny = 0.0;
        for (std::size_t i = 1; i < points_size; ++i) {
            nx = xy_points[i].first;
            ny = xy_points[i].second;
            double end_segment_s = std::sqrt((fx - nx) * (fx - nx) + (fy - ny) * (fy - ny));
            accumulated_s->push_back(end_segment_s + distance);
            distance += end_segment_s;
            fx = nx;
            fy = ny;
        }

        // Get finite difference approximated first derivative of y and x respective
        // to s for kappa calculation
        for (std::size_t i = 0; i < points_size; ++i) {
            double xds = 0.0;
            double yds = 0.0;
            if (i == 0) {
                xds = (xy_points[i + 1].first - xy_points[i].first) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
                yds = (xy_points[i + 1].second - xy_points[i].second) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i));
            } else if (i == points_size - 1) {
                xds = (xy_points[i].first - xy_points[i - 1].first) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
                yds = (xy_points[i].second - xy_points[i - 1].second) /
                      (accumulated_s->at(i) - accumulated_s->at(i - 1));
            } else {
                xds = (xy_points[i + 1].first - xy_points[i - 1].first) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
                yds = (xy_points[i + 1].second - xy_points[i - 1].second) /
                      (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            x_over_s_first_derivatives.push_back(xds);
            y_over_s_first_derivatives.push_back(yds);
        }

        // Get finite difference approximated second derivative of y and x respective
        // to s for kappa calculation
        for (std::size_t i = 0; i < points_size; ++i) {
            double xdds = 0.0;
            double ydds = 0.0;
            if (i == 0) {
                xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i]) /
                       (accumulated_s->at(i + 1) - accumulated_s->at(i));
                ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i]) /
                       (accumulated_s->at(i + 1) - accumulated_s->at(i));
            } else if (i == points_size - 1) {
                xdds = (x_over_s_first_derivatives[i] - x_over_s_first_derivatives[i - 1]) /
                       (accumulated_s->at(i) - accumulated_s->at(i - 1));
                ydds = (y_over_s_first_derivatives[i] - y_over_s_first_derivatives[i - 1]) /
                       (accumulated_s->at(i) - accumulated_s->at(i - 1));
            } else {
                xdds = (x_over_s_first_derivatives[i + 1] - x_over_s_first_derivatives[i - 1]) /
                       (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
                ydds = (y_over_s_first_derivatives[i + 1] - y_over_s_first_derivatives[i - 1]) /
                       (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            x_over_s_second_derivatives.push_back(xdds);
            y_over_s_second_derivatives.push_back(ydds);
        }

        for (std::size_t i = 0; i < points_size; ++i) {
            double xds = x_over_s_first_derivatives[i];
            double yds = y_over_s_first_derivatives[i];
            double xdds = x_over_s_second_derivatives[i];
            double ydds = y_over_s_second_derivatives[i];
            double kappa =
                    (xds * ydds - yds * xdds) / (std::sqrt(xds * xds + yds * yds) * (xds * xds + yds * yds) + 1e-6);
            kappas->push_back(kappa);
        }

        // Dkappa calculation
        for (std::size_t i = 0; i < points_size; ++i) {
            double dkappa = 0.0;
            if (i == 0) {
                dkappa = (kappas->at(i + 1) - kappas->at(i)) / (accumulated_s->at(i + 1) - accumulated_s->at(i));
            } else if (i == points_size - 1) {
                dkappa = (kappas->at(i) - kappas->at(i - 1)) / (accumulated_s->at(i) - accumulated_s->at(i - 1));
            } else {
                dkappa =
                        (kappas->at(i + 1) - kappas->at(i - 1)) / (accumulated_s->at(i + 1) - accumulated_s->at(i - 1));
            }
            dkappas->push_back(dkappa);
        }
        return true;
    }

}

