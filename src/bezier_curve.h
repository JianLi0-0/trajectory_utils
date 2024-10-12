#ifndef SRC_BEZIER_CURVE_H
#define SRC_BEZIER_CURVE_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "tf/transform_datatypes.h"
#include "Eigen/Geometry"

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& robot_pose, const geometry_msgs::Pose& target_pose, const bool inverse) {
    // 提取机器人位姿的平移和旋转部分
    Eigen::Translation3d world_to_robot_t(robot_pose.position.x, robot_pose.position.y, robot_pose.position.z);
    Eigen::Quaterniond world_to_robot_r(robot_pose.orientation.w, robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z);
    // 创建变换矩阵
    Eigen::Affine3d world_to_robot_tf = world_to_robot_t * world_to_robot_r;

    // 提取机器人位姿的平移和旋转部分
    Eigen::Translation3d target_t(target_pose.position.x, target_pose.position.y, target_pose.position.z);
    Eigen::Quaterniond target_r(target_pose.orientation.w, target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z);
    // 创建变换矩阵
    Eigen::Affine3d target_tf = target_t * target_r;

    // 将目标位姿的平移部分变换到机器人坐标系
    Eigen::Affine3d robot_to_target_tf;
    if (inverse) {
        robot_to_target_tf = world_to_robot_tf.inverse() * target_tf; // target_tf is world_to_target_tf
    } else {
        robot_to_target_tf = world_to_robot_tf * target_tf; // target_tf is robot_to_target_tf
    }

    // 创建变换后的位姿
    geometry_msgs::Pose transformed_pose;
    transformed_pose.position.x = robot_to_target_tf.translation().x();
    transformed_pose.position.y = robot_to_target_tf.translation().y();
    transformed_pose.position.z = robot_to_target_tf.translation().z();
    Eigen::Quaterniond q(robot_to_target_tf.rotation());
    transformed_pose.orientation.w = q.w();
    transformed_pose.orientation.x = q.x();
    transformed_pose.orientation.y = q.y();
    transformed_pose.orientation.z = q.z();

    return transformed_pose;
}


double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return std::sqrt(dx*dx+dy*dy);
}

// 计算二项式系数的函数
unsigned long binomialCoefficient(unsigned int n, unsigned int k) {
    if (k > n) return 0;
    if (k == 0 || k == n) return 1;
    if (k > n - k) k = n - k;
    unsigned long c = 1;
    for (unsigned int i = 0; i < k; ++i) {
        c *= (n - i);
        c /= (i + 1);
    }
    return c;
}

// 计算贝塞尔曲线点的函数
std::vector<geometry_msgs::PoseStamped> bezierCurve(
        const std::vector<geometry_msgs::PoseStamped>& points, const double& interval) {
    unsigned int numPoints = 100;
    auto len = distance(points.front(), points.back());
    numPoints = static_cast<unsigned int>(len / interval);
    std::vector<geometry_msgs::PoseStamped> curve(numPoints);
    unsigned int n = points.size() - 1;

    // 预计算二项式系数
    std::vector<unsigned long> binomials(n + 1);
    for (unsigned int i = 0; i <= n; ++i) {
        binomials[i] = binomialCoefficient(n, i);
    }

    // 计算贝塞尔曲线
    for (unsigned int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        float one_minus_t = 1.0 - t;
        float x = 0.0;
        float y = 0.0;
        for (unsigned int j = 0; j <= n; ++j) {
            float coefficient = binomials[j] * std::pow(one_minus_t, n - j) * std::pow(t, j);
            x += points[j].pose.position.x * coefficient;
            y += points[j].pose.position.y * coefficient;
        }
        curve[i].pose.position.x = x;
        curve[i].pose.position.y = y;
        curve[i].pose.orientation.w = 1;
        curve[i].header.frame_id = points[0].header.frame_id;
    }

    return curve;
}

std::vector<geometry_msgs::PoseStamped> cubicBezierCurve(
        const std::vector<geometry_msgs::PoseStamped>& points) {
    auto len = distance(points.front(), points.back());
    geometry_msgs::PoseStamped p1, p2, p3, p4;
    p1 = points.front();
    p4 = points.back();

    // 起点向前延伸0.35倍的距离
    p2 = p1;
    double cof_start = 1.2;
    p2.pose.position.x += cof_start * len * std::cos(tf::getYaw(p1.pose.orientation));
    p2.pose.position.y += cof_start * len * std::sin(tf::getYaw(p1.pose.orientation));

    // 终点向后延伸0.4倍的距离
    p3 = p4;
    double cof_end = 0.9;
    p3.pose.position.x += cof_end * (p1.pose.position.x - p4.pose.position.x);
    p3.pose.position.y += cof_end * (p1.pose.position.y - p4.pose.position.y);

    return bezierCurve({p1,p2,p3,p4}, 0.05);
}

// 四阶贝塞尔曲线，可约束起始曲率
std::vector<geometry_msgs::PoseStamped> kappaConstrainedBezierCurve(
        const std::vector<geometry_msgs::PoseStamped>& points, const double& kappa_start) {

    geometry_msgs::PoseStamped r_p0, r_p1, r_p2, r_p3, r_p4; // 机器人坐标系下的控制点
    r_p0.pose.orientation.w = 1.0;
    r_p4.pose = transformPose(points.front().pose, points.back().pose, true);

    cout << "transformPose(points.front().pose, points.back().pose, true): " << transformPose(points.front().pose, points.back().pose, true);
    cout << "r_p4: " << r_p4;

    double len = distance(points.front(), points.back());
    double d1 = 0.25 * len;
    double d4 = 0.1 * len;
    double x2 = r_p0.pose.position.x + 0.7 * (r_p4.pose.position.x - r_p0.pose.position.x);

    r_p1 = r_p0;
    r_p1.pose.position.x = d1;
    r_p1.pose.position.y = r_p0.pose.position.y;

    r_p2 = r_p0;
    r_p2.pose.position.x = x2;
    r_p2.pose.position.y = 4.0 / 3.0 * kappa_start * d1 * d1;

    // 终点向后延伸0.25倍的距离
    r_p3 = r_p0;
    r_p3.pose.position.x = r_p4.pose.position.x - d4 * std::cos(tf::getYaw(r_p4.pose.orientation));
    r_p3.pose.position.y = r_p4.pose.position.y - d4 * std::sin(tf::getYaw(r_p4.pose.orientation));

    geometry_msgs::PoseStamped p0, p1, p2, p3, p4;
    p0 = points.front();
    p1.pose = transformPose(p0.pose, r_p1.pose, false);
    p2.pose = transformPose(p0.pose, r_p2.pose, false);
    p3.pose = transformPose(p0.pose, r_p3.pose, false);
    p4 = points.back();

    return bezierCurve({p0,p1,p2,p3,p4}, 0.05);
}


#endif //SRC_BEZIER_CURVE_H
