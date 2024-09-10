# include "trajectory_info.h"
#include "matplotlibcpp.h"

using namespace trajectory_utils;
namespace plt = matplotlibcpp;

int main(int argc, char **argv) {
    ros::init(argc, argv, "trajectory_info_test");
    ros::NodeHandle nh;

    TrajectoryInfo trajectory_info;
    std::vector<PathPoint> path_data;
    double l1 = 1.0;
    double r1 = 1.5;
    double l2 = 0.5;
    for (double l = 0.0; l < l1; l += 0.1) {
        PathPoint p;
        p.set_x(l);
        p.set_y(0.0);
        path_data.push_back(p);
    }
    for (double rad = -M_PI / 2; rad < M_PI; rad += 0.1) {
        PathPoint p;
        p.set_x(l1 + r1 * cos(rad));
        p.set_y(r1 + r1 * sin(rad));
        path_data.push_back(p);
    }
    for (double l = 0.0; l < l2; l += 0.1) {
        PathPoint p;
        p.set_x(l1 - r1);
        p.set_y(r1 - l);
        path_data.push_back(p);
    }

    ros::Time start_time = ros::Time::now();
    trajectory_info.setPathData(path_data);
    trajectory_info.calSpeedData(
            0.0, 0.0, 0.0, trajectory_info.getPathDataPtr()->Length());

    trajectory_info.combinePathAndSpeedProfile();
    auto discretized_trajectory = trajectory_info.getTrajectoryPtr();
    TrajectoryPoint traj_point;
    trajectory_info.getRefTrajectoryPoint(Vec2d(1.0, -0.2), traj_point);
    std::cout << traj_point.DebugString() << std::endl;
    ROS_INFO("Time elapsed: %f", (ros::Time::now() - start_time).toSec());

    std::vector<double> x, y;
    for (const auto &point : path_data) {
        x.push_back(point.x());
        y.push_back(point.y());
    }

    plt::figure_size(640, 640);
    plt::xlim(-2, 5);
    plt::ylim(-2, 5);
    plt::plot(x, y);
    plt::title("Path");
    plt::grid(true);

    std::vector<double> t_vec, s_vec, v_vec, a_vec;
    std::array<double, 1> s, v, a;
    double time_span = trajectory_info.getSpeedDataPtr()->get_duration();
    for (double t=0.0;t<=time_span;t+=0.01) {
        trajectory_info.getSpeedDataPtr()->at_time(t, s, v, a);
        t_vec.push_back(t);
        s_vec.push_back(s[0]);
        v_vec.push_back(v[0]);
        a_vec.push_back(a[0]);
    }

    plt::figure_size(640, 640);
    plt::plot(t_vec, s_vec);
    plt::plot(t_vec, v_vec);
    plt::plot(t_vec, a_vec);
    plt::title("Speed Profile");
    plt::grid(true);

    std::vector<double> x_traj_vec, y_traj_vec, s_traj_vec, v_traj_vec, a_traj_vec, kappa_traj_vec;
    x_traj_vec.reserve(t_vec.size());
    y_traj_vec.reserve(t_vec.size());
    s_traj_vec.reserve(t_vec.size());
    v_traj_vec.reserve(t_vec.size());
    a_traj_vec.reserve(t_vec.size());
    kappa_traj_vec.reserve(t_vec.size());
    for (auto t : t_vec) {
        auto traj_point = discretized_trajectory->Evaluate(t);
        x_traj_vec.push_back(traj_point.path_point().x());
        y_traj_vec.push_back(traj_point.path_point().y());
        s_traj_vec.push_back(traj_point.path_point().s());
        kappa_traj_vec.push_back(traj_point.path_point().kappa());
        v_traj_vec.push_back(traj_point.v());
        a_traj_vec.push_back(traj_point.a());
    }

//    plt::figure_size(640, 640);
    plt::plot(t_vec, s_traj_vec);
    plt::plot(t_vec, v_traj_vec);
    plt::plot(t_vec, a_traj_vec);
    plt::plot(t_vec, kappa_traj_vec);
//    plt::plot(x_traj_vec, y_traj_vec);
    plt::title("Traj Profile");
    plt::grid(true);


    plt::show();


//        ros::Rate rate(10);

//        while (ros::ok()) {
//            ros::spinOnce();
//            ROS_INFO("Trajectory info test.");
//            rate.sleep();
//        }

    return 0;
}
