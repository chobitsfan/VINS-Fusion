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

//#define SEND_FEATURES
static struct sockaddr_un chobits_addr, chobits_local_addr;
static int chobits_sock;
#ifdef LOG_FEATURES
extern FILE* my_log_file2;
extern int my_log_num;
#endif
std::shared_ptr<rclcpp::Node> node;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odo_pub;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br;

void registerPub()
{
    memset(&chobits_addr, 0, sizeof(struct sockaddr_un));
    chobits_addr.sun_family = AF_UNIX;
    strcpy(chobits_addr.sun_path, "/tmp/chobits_server");
    memset(&chobits_local_addr, 0, sizeof(struct sockaddr_un));
    chobits_local_addr.sun_family = AF_UNIX;
    strcpy(chobits_local_addr.sun_path, "/tmp/chobits_1234");
    chobits_sock = socket(AF_UNIX, SOCK_DGRAM, 0);
    unlink("/tmp/chobits_1234");
    bind(chobits_sock, (struct sockaddr*)&chobits_local_addr, sizeof(chobits_local_addr));

    node = rclcpp::Node::make_shared("vins");
    odo_pub = node->create_publisher<nav_msgs::msg::Odometry>("odometry", 1);
    tf_br = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

void pubOdometry(const Estimator &estimator)
{
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

        float chobits_msg[10] = { (float)qw, (float)qx, (float)qy, (float)qz, (float)px, (float)py, (float)pz, (float)vx, (float)vy, (float)vz };
        sendto(chobits_sock, chobits_msg, sizeof(chobits_msg), 0, (struct sockaddr*)&chobits_addr, sizeof(chobits_addr));

        std_msgs::msg::Header header;
        header.stamp = node->get_clock()->now();
        header.frame_id = "map";
        geometry_msgs::msg::TransformStamped tf;
        tf.header = header;
        tf.child_frame_id = "body";
        tf.transform.translation.x = px;
        tf.transform.translation.y = py;
        tf.transform.translation.z = pz;
        tf.transform.rotation.x = qx;
        tf.transform.rotation.y = qy;
        tf.transform.rotation.z = qz;
        tf.transform.rotation.w = qw;
        tf_br->sendTransform(tf);

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
        odo_msg.twist.twist.linear.x = vy;
        odo_msg.twist.twist.linear.x = vz;
        odo_pub->publish(odo_msg);
#ifdef LOG_FEATURES
        fprintf(my_log_file2, "%d,%f,%f,%f\n", my_log_num, px, py, pz);
#endif
        //fprintf(my_log_file, ",%f,%f,%f\n", px, py, pz);
    }
#ifdef SEND_FEATURES
    if (pub_addr.sin_family == AF_INET) {
        float pp_msg[40*3+1];
        float* pp_msg_ptr = pp_msg;
        int c = 0;
        for (auto &it_per_id : estimator.f_manager.feature)
        {
            int used_num;
            used_num = it_per_id.feature_per_frame.size();
            if (!(used_num >= 2 && it_per_id.start_frame < WINDOW_SIZE - 2))
                continue;
            if (it_per_id.start_frame > WINDOW_SIZE * 3.0 / 4.0 || it_per_id.solve_flag != 1)
                continue;
            int imu_i = it_per_id.start_frame;
            Vector3d pts_i = it_per_id.feature_per_frame[0].point * it_per_id.estimated_depth;
            Vector3d w_pts_i = estimator.Rs[imu_i] * (estimator.ric[0] * pts_i + estimator.tic[0]) + estimator.Ps[imu_i];
            *++pp_msg_ptr = w_pts_i(0);
            *++pp_msg_ptr = w_pts_i(1);
            *++pp_msg_ptr = w_pts_i(2);
            ++c;
            if (c >= 40) break;
        }
        pp_msg[0] = c;
        sendto(pub_sock, pp_msg, sizeof(pp_msg), 0, (struct sockaddr*)&pub_addr, sizeof(pub_addr));
    }
#endif
}
