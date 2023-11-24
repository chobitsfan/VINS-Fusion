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
#include <time.h>
#include <cuda_runtime.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudaarithm.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

int gogogo = true;

void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    //printf("getImageFromMsg\n");
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}

// extract images with same timestamp from two topics
void sync_process()
{
    void *unified_ptr;
    cudaMallocManaged(&unified_ptr, 1280*400);
    cv::Mat frame(400, 1280, CV_8UC1, unified_ptr);
    cv::cuda::GpuMat g_frame(400, 1280, CV_8UC1, unified_ptr);
    cv::cuda::GpuMat g_frame_l_rect(400, 640, CV_8UC1);
    cv::cuda::GpuMat g_frame_r_rect(400, 640, CV_8UC1);

    cv::Mat cam0 = (cv::Mat_<double>(3,3) << 4.5871895544956595e+02, 0., 3.3657517879988706e+02, 0.,4.5873147178036533e+02, 2.1628073654244133e+02, 0., 0., 1.);
    cv::Mat dist0 = (cv::Mat_<double>(5,1) << 5.7076449331591138e-02, -4.7450762182942974e-02, -3.3333501353066592e-03, 3.5795258190291261e-04, 0.);
    cv::Mat cam1 = (cv::Mat_<double>(3,3) << 4.5812706153237770e+02, 0., 3.2301901906538336e+02, 0., 4.5787818725258467e+02, 2.2334506060755325e+02, 0., 0., 1.);
    cv::Mat dist1 = (cv::Mat_<double>(5,1) << 5.3889271815474718e-02, -5.3415172791704893e-02, -4.7030040966682882e-03, 1.0284987824595350e-03, 0.);
    cv::Mat R1 =  (cv::Mat_<double>(3,3) << 9.9994869126998354e-01, -7.3895828685681345e-03,
       -6.9288449597343840e-03, 7.4216277987357877e-03,
       9.9996183013404594e-01, 4.6106090472299924e-03,
       6.8945100090219780e-03, -4.6617957911014750e-03,
       9.9996536609611519e-01);
    cv::Mat R2 = (cv::Mat_<double>(3,3) << 9.9986633282874682e-01, -9.8602871037420207e-03,
       1.3041902231865726e-02, 9.9206488179775787e-03,
       9.9994033826988116e-01, -4.5717204361087086e-03,
       -1.2996045653356457e-02, 4.7004934791309884e-03,
       9.9990449951904337e-01);
    cv::Mat P1 = (cv::Mat_<double>(3,4) << 4.9192896209865114e+02, 0., 3.2796942901611328e+02, 0., 0.,
       4.9192896209865114e+02, 2.1781097984313965e+02, 0., 0., 0., 1., 0.);
    cv::Mat P2 = (cv::Mat_<double>(3,4) << 4.9192896209865114e+02, 0., 3.2796942901611328e+02,
       -4.9456318369509859e+02, 0., 4.9192896209865114e+02,
       2.1781097984313965e+02, 0., 0., 0., 1., 0.);
    cv::Mat Q = (cv::Mat_<float>(4,4) << 1., 0., 0., -3.2796942901611328e+02, 0., 1., 0.,
       -2.1781097984313965e+02, 0., 0., 0., 4.9192896209865114e+02, 0.,
       0., 9.9467363992449664e-01, 0.);
    cv::Mat cam0_map1, cam0_map2, cam1_map1, cam1_map2;
    cv::initUndistortRectifyMap(cam0, dist0, R1, P1, cv::Size2i(640,400), CV_32FC1, cam0_map1, cam0_map2);
    cv::initUndistortRectifyMap(cam1, dist1, R2, P2, cv::Size2i(640,400), CV_32FC1, cam1_map1, cam1_map2);
    cv::cuda::GpuMat g_cam0_map1(cam0_map1);
    cv::cuda::GpuMat g_cam0_map2(cam0_map2);
    cv::cuda::GpuMat g_cam1_map1(cam1_map1);
    cv::cuda::GpuMat g_cam1_map2(cam1_map2);

    bool init_fps=true;
    cv::VideoCapture cap;
    cap.open("/dev/video0", cv::CAP_V4L2);
    while (!cap.isOpened()) {
        printf("can not open camera, try again...\n");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cap.open("/dev/video0", cv::CAP_V4L2);
    }
    printf("camera opened\n");
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('G','R','E','Y'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 400);
    cap.set(cv::CAP_PROP_CONVERT_RGB, 0);
    //cap.set(cv::CAP_PROP_FPS, 30); //do not work

    double time = 0;
    struct timespec ts;
    while (gogogo)
    {
        if (cap.grab()) {
            clock_gettime(CLOCK_REALTIME, &ts);
            time = ts.tv_sec + (double)ts.tv_nsec / 1e9;
            cap.retrieve(frame);
            if (init_fps) {
                init_fps=false;
                system("v4l2-ctl -c exposure=900,frame_rate=30");
            } else {
                cv::cuda::GpuMat g_frame_l = g_frame.colRange(g_frame.cols / 2, g_frame.cols);
                cv::cuda::GpuMat g_frame_r = g_frame.colRange(0, g_frame.cols / 2);
                cv::cuda::remap(g_frame_l, g_frame_l_rect, g_cam0_map1, g_cam0_map2, cv::INTER_LINEAR);
                cv::cuda::remap(g_frame_r, g_frame_r_rect, g_cam1_map1, g_cam1_map2, cv::INTER_LINEAR);
                estimator.inputImage(time, g_frame_l_rect, g_frame_r_rect);
            }
        }
    }

    cudaFree(&unified_ptr);
    cap.release();
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
{
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        ROS_ASSERT(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.toSec();
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::BoolConstPtr &restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void send_apm_callback(const std_msgs::BoolConstPtr &msg) {
    ROS_WARN("send pose to apm\n");
    estimator.send_pose_apm = msg->data;
}

void imu_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::BoolConstPtr &switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
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

    std::thread sync_thread{sync_process};

    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 200, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);
    ros::Subscriber sub_send_apm = n.subscribe("/vins_send_pose_apm", 1, send_apm_callback);

    ros::spin();

    printf("game over\n");
    gogogo = false;
    estimator.gogogo = false;
    sync_thread.join(); 

    return 0;
}
