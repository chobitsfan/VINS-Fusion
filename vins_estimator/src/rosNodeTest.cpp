/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <ros/ros.h>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#define IMU_SOCK_PATH "/tmp/chobits_imu"
#define FEATURES_SOCK_PATH "/tmp/chobits_features"

#define BUF_SZ 12*1024

double buf[BUF_SZ/sizeof(double)];

void sig_func(int sig) {}

int main(int argc, char **argv)
{
    Estimator estimator;

    struct pollfd pfds[2];
    int imu_sock, features_sock;
    struct sockaddr_un ipc_addr;
    if ((imu_sock = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, IMU_SOCK_PATH);
    unlink(IMU_SOCK_PATH);
    if (bind(imu_sock, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        printf("bind local failed\n");
        return 1;
    }

    if ((features_sock = socket(AF_UNIX, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&ipc_addr, 0, sizeof(ipc_addr));
    ipc_addr.sun_family = AF_UNIX;
    strcpy(ipc_addr.sun_path, FEATURES_SOCK_PATH);
    unlink(FEATURES_SOCK_PATH);
    if (bind(features_sock, (const struct sockaddr *)&ipc_addr, sizeof(ipc_addr)) < 0) {
        printf("bind local failed\n");
        return 1;
    }
    pfds[0].fd= imu_sock;
    pfds[0].events = POLLIN;
    pfds[1].fd= features_sock;
    pfds[1].events = POLLIN;

    signal(SIGINT, sig_func);

    ros::init(argc, argv, "vins_estimator", ros::init_options::NoSigintHandler);
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
    while (true) {
        if (poll(pfds, 2, -1) > 0) {
            if (pfds[0].revents & POLLIN) {
                if (recv(imu_sock, buf, BUF_SZ, 0) > 0) {
                    Vector3d acc(buf[1], buf[2], buf[3]);
                    Vector3d gyr(buf[4], buf[5], buf[6]);
                    estimator.inputIMU(buf[0], acc, gyr);
                }
            }
            if (pfds[1].revents & POLLIN) {
                int len = recv(features_sock, buf, BUF_SZ, 0);
                if (len > 0) {
                    double* features_data = buf;
                    int num = features_data[0];
                    //printf("rcv %d bytes, %d features\n", len, num);
                    double t = features_data[1];
                    features_data += 2;
                    featureFrame.clear();
                    for (int i = 0; i < num; ++i) {
                        int id = features_data[0];
                        xyz_uv_velocity << features_data[1], features_data[2], 1, features_data[3], features_data[4], features_data[5], features_data[6];
                        featureFrame[id].emplace_back(0,  xyz_uv_velocity);
                        xyz_uv_velocity << features_data[7], features_data[8], 1, features_data[9], features_data[10], features_data[11], features_data[12];
                        featureFrame[id].emplace_back(1,  xyz_uv_velocity);
                        features_data += 13;
                    }
                    estimator.inputFeature(t, featureFrame);
                }
            }
        } else break;
    }

    estimator.gogogo = false;

    unlink(IMU_SOCK_PATH);
    unlink(FEATURES_SOCK_PATH);

    printf("bye\n");

    return 0;
}
