/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "visualization.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <netinet/udp.h>
#include <arpa/inet.h>
#include <errno.h>
#include <stdio.h>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define MAX_TF_QUEUE_SIZE 40

#ifdef LOG_FEATURES
extern FILE* my_log_file2;
extern int my_log_num;
#endif

extern bool gogogo;
std::mutex tf_mtx;
std::condition_variable tf_cv;
std::deque<geometry_msgs::msg::TransformStamped> tf_queue;

void pub_result_func(const Estimator* estimator) {
    while (true) {
        std::deque<geometry_msgs::msg::TransformStamped> local_queue;
        {
            std::unique_lock<std::mutex> lock(tf_mtx);
            tf_cv.wait(lock, [] { return !tf_queue.empty() || !gogogo; }); // Wait until queue has items OR should exit
            if (!gogogo) return;
            // Grab ALL pending work
            local_queue.swap(tf_queue);
        } // Automatic unlock
        // Process all items without holding lock
        while (!local_queue.empty()) {
            estimator->tf_br->sendTransform(local_queue.front()); // sendTransform may take some time
            local_queue.pop_front();
        }
    }
}

void registerPub(Estimator &estimator)
{
    estimator.ros_node = rclcpp::Node::make_shared("vins");
    estimator.odo_pub = estimator.ros_node->create_publisher<nav_msgs::msg::Odometry>("odometry", rclcpp::QoS(1).best_effort().durability_volatile());
    estimator.ft_pub = estimator.ros_node->create_publisher<sensor_msgs::msg::PointCloud>("features", rclcpp::QoS(1).best_effort().durability_volatile());
#ifdef PUB_TRACK
    estimator.track_pub = estimator.ros_node->create_publisher<visualization_msgs::msg::Marker>("track", rclcpp::QoS(1).best_effort().durability_volatile());
#endif
    estimator.tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(estimator.ros_node);
}

void pubOdometry(const Estimator &estimator, const double feature_ts)
{
#ifdef PUB_TRACK
    static unsigned int path_c = 0;
    static unsigned int path_i = 0;
    static double prv_px = 0;
    static double prv_py = 0;
    static double prv_pz = 0;
#endif
    std_msgs::msg::Header header;
    //struct timespec tp;
    //clock_gettime(CLOCK_MONOTONIC, &tp);
    //printf("cost %ld\n", (int64_t)tp.tv_sec * 1000000000 + tp.tv_nsec - (int64_t)(feature_ts * 1000000000));
    //header.stamp = rclcpp::Time(feature_ts * 1000000000, RCL_STEADY_TIME);
    header.stamp.sec = (int32_t)feature_ts;
    header.stamp.nanosec = (uint32_t)((feature_ts - (int32_t)feature_ts) * 1000000000);
    header.frame_id = "map";
    if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
    {
        double px = estimator.Ps[WINDOW_SIZE].x();
        double py = estimator.Ps[WINDOW_SIZE].y();
        double pz = estimator.Ps[WINDOW_SIZE].z();
        double vx = estimator.Vs[WINDOW_SIZE].x();
        double vy = estimator.Vs[WINDOW_SIZE].y();
        double vz = estimator.Vs[WINDOW_SIZE].z();

        Eigen::Quaterniond q = Eigen::Quaterniond(estimator.Rs[WINDOW_SIZE]);
        double qx = q.x();
        double qy = q.y();
        double qz = q.z();
        double qw = q.w();

        geometry_msgs::msg::TransformStamped tf_to_pub;
        tf_to_pub.header = header;
        tf_to_pub.child_frame_id = "body";
        tf_to_pub.transform.translation.x = px;
        tf_to_pub.transform.translation.y = py;
        tf_to_pub.transform.translation.z = pz;
        tf_to_pub.transform.rotation.x = qx;
        tf_to_pub.transform.rotation.y = qy;
        tf_to_pub.transform.rotation.z = qz;
        tf_to_pub.transform.rotation.w = qw;
        {
            std::lock_guard<std::mutex> lock(tf_mtx);
            if (tf_queue.size() >= MAX_TF_QUEUE_SIZE) tf_queue.pop_front();
            tf_queue.push_back(tf_to_pub);
            tf_cv.notify_one(); // wake up worker
        } // Hold locks for the shortest time possible

        nav_msgs::msg::Odometry odo_msg;
        odo_msg.header = header;
        odo_msg.child_frame_id = "map";
        odo_msg.pose.pose.position.x = px;
        odo_msg.pose.pose.position.y = py;
        odo_msg.pose.pose.position.z = pz;
        odo_msg.pose.pose.orientation.x= qx;
        odo_msg.pose.pose.orientation.y = qy;
        odo_msg.pose.pose.orientation.z = qz;
        odo_msg.pose.pose.orientation.w = qw;
        odo_msg.twist.twist.linear.x = vx;
        odo_msg.twist.twist.linear.y = vy;
        odo_msg.twist.twist.linear.z = vz;
        estimator.odo_pub->publish(odo_msg);

#if PUB_TRACK
        path_c++;
        if (path_c > 5) {
            path_c = 0;
            visualization_msgs::msg::Marker line_list;
            line_list.header = header;
            line_list.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_list.action = visualization_msgs::msg::Marker::ADD;
            line_list.pose.orientation.w = 1.0;
            line_list.id = path_i;
            line_list.ns = "track";
            line_list.scale.x = 0.02;
            line_list.color.r = 1.0;
            line_list.color.g = 1.0;
            line_list.color.b = 1.0;
            line_list.color.a = 1.0;
            geometry_msgs::msg::Point p;
            p.x = prv_px;
            p.y = prv_py;
            p.z = prv_pz;
            line_list.points.push_back(p);
            p.x = px;
            p.y = py;
            p.z = pz;
            line_list.points.push_back(p);
            estimator.track_pub->publish(line_list);
            path_i++;
            prv_px = px;
            prv_py = py;
            prv_pz = pz;
        }
#endif
#ifdef LOG_FEATURES
        fprintf(my_log_file2, "%d,%f,%f,%f\n", my_log_num, px, py, pz);
#endif
        //fprintf(my_log_file, ",%f,%f,%f\n", px, py, pz);
    }
    sensor_msgs::msg::PointCloud features;
    features.header = header;
    for (auto &it_per_id : estimator.f_manager.feature) {
        int used_num;
        used_num = it_per_id.feature_per_frame.size();
        if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
            continue;
        if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
            continue;
        int imu_i = it_per_id.start_frame;
        Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
        Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
        geometry_msgs::msg::Point32 p;
        p.x = w_pts_i(0);
        p.y = w_pts_i(1);
        p.z = w_pts_i(2);
        features.points.push_back(p);
    }
    estimator.ft_pub->publish(features);
}
