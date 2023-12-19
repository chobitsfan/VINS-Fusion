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
#include <condition_variable>
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
#include <opencv2/cudafilters.hpp>
//#include <vpi/OpenCVInterop.hpp> 
//#include <vpi/Image.h>
//#include <vpi/Status.h>
//#include <vpi/Stream.h>
//#include <vpi/algo/ConvertImageFormat.h>
//#include <vpi/algo/StereoDisparity.h>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

Estimator estimator;

//queue<sensor_msgs::ImageConstPtr> img0_buf;
//queue<sensor_msgs::ImageConstPtr> img1_buf;
//std::mutex m_buf;

static ros::Publisher depth_img_pub;

bool gogogo = true;

std::mutex depth_m;
std::condition_variable depth_cv;
bool img_ready=false;
cv::cuda::GpuMat g_frame_l_rect(400, 640, CV_8UC1);
cv::cuda::GpuMat g_frame_r_rect(400, 640, CV_8UC1);

/*void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
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
}*/


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

void depth_process() {
    double baseline = 1.0077095772657289e-01;
    double focal = 4.9297293111444242e+02;
    cv::cuda::Stream cuda_stream;
    cv::Ptr<cv::cuda::StereoSGM> sgm = cv::cuda::createStereoSGM(0, 128, 10, 120, 15);
    cv::Mat open_k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    cv::Ptr<cv::cuda::Filter> morph_filter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_32FC1, open_k);
    cv::cuda::GpuMat g_disp_map;
    cv::cuda::GpuMat g_disp_map_th;
    cv::cuda::GpuMat g_disp_map_scaled;
    cv::cuda::GpuMat g_disp_map_filtered;
    cv::cuda::GpuMat g_disp_map_dsz;
    cv::Mat disp_map;
    cv::Mat depth_img(400,640,CV_16UC1);
    struct timespec ts;    
    std_msgs::Header ros_header;
    ros_header.frame_id = "image";
    std::unique_lock<std::mutex> lk(depth_m);
    std::this_thread::sleep_for(3s);
    while (gogogo) {
        if (depth_cv.wait_for(lk, 600ms,[]{ return img_ready; })) {
            img_ready=false;
#if 1
            clock_gettime(CLOCK_REALTIME, &ts);
            ros_header.stamp.sec = ts.tv_sec;
            ros_header.stamp.nsec = ts.tv_nsec;
            sgm->compute(g_frame_l_rect, g_frame_r_rect, g_disp_map, cuda_stream);
            cv::cuda::resize(g_disp_map, g_disp_map_dsz, cv::Size(320,200), 0, 0, cv::INTER_LINEAR, cuda_stream);
            cv::cuda::threshold(g_disp_map_dsz, g_disp_map_th, 0, 0, cv::THRESH_TOZERO, cuda_stream); //sgm have negative disp
            g_disp_map_th.convertTo(g_disp_map_scaled, CV_32FC1, 1.0/16.0, cuda_stream); //sgm have 4 fractional bits
            morph_filter->apply(g_disp_map_scaled, g_disp_map_filtered, cuda_stream);
            g_disp_map_filtered.download(disp_map);
            cv::divide(baseline*focal*1000, disp_map, depth_img, CV_16UC1); //cuda::divide work different
            depth_img_pub.publish(cv_bridge::CvImage(ros_header, "mono16", depth_img).toImageMsg());
#endif
        } else break;
    }

    cout << "depth process end\n";
}

// extract images with same timestamp from two topics
void sync_process()
{
#if 1     
    void *unified_ptr;
    cudaMallocManaged(&unified_ptr, 1280*400);
    cv::Mat frame(400, 1280, CV_8UC1, unified_ptr);
    cv::cuda::GpuMat g_frame(400, 1280, CV_8UC1, unified_ptr);
    cv::cuda::GpuMat g_frame_l, g_frame_r;
#else
    cv::Mat frame(400, 1280, CV_8UC1);
    cv::cuda::GpuMat g_frame(400, 1280, CV_8UC1);
    cv::cuda::GpuMat g_frame_l, g_frame_r;
    cv::cuda::GpuMat g_frame_l_rect(400, 640, CV_8UC1);
    cv::cuda::GpuMat g_frame_r_rect(400, 640, CV_8UC1);
#endif

    cv::Mat cam0 = (cv::Mat_<double>(3,3) <<
        4.5871894208851108e+02, 0., 3.3657516241366756e+02, 0.,
        4.5873145101846643e+02, 2.1628074178739521e+02, 0., 0., 1.);
    cv::Mat dist0 = (cv::Mat_<double>(5,1) << 
        5.7076163517889071e-02, -4.7449852647679529e-02,
       -3.3333133411082233e-03, 3.5791943710580180e-04, 0.);
    cv::Mat cam1 = (cv::Mat_<double>(3,3) << 
        4.5835888364887256e+02, 0., 3.2070900760895131e+02, 0.,
        4.5836189927937050e+02, 2.2369740035407133e+02, 0., 0., 1.);
    cv::Mat dist1 = (cv::Mat_<double>(5,1) << 
        5.9085906436082768e-02, -6.2062049999354461e-02,
       -4.2127875324890858e-03, -4.0428396364065671e-04, 0.);
    cv::Mat R1 =  (cv::Mat_<double>(3,3) <<
        9.9992050723396342e-01, -7.5619535204292680e-03,
       -1.0089403943158532e-02, 7.6080712339965124e-03,
       9.9996075032904486e-01, 4.5403803223814567e-03,
       1.0054673792410714e-02, -4.6167802988988796e-03,
       9.9993879256412488e-01);
    cv::Mat R2 = (cv::Mat_<double>(3,3) << 
        9.9990719408534878e-01, -1.0043377142692959e-02,
       9.2050959763823655e-03, 1.0085419948126504e-02,
       9.9993886915486030e-01, -4.5323567565640306e-03,
       -9.1590130928343328e-03, 4.6247733856343461e-03,
       9.9994736058969469e-01);
    cv::Mat P1 = (cv::Mat_<double>(3,4) <<
        4.9297293111444242e+02, 0., 3.2864122009277344e+02, 0., 0.,
       4.9297293111444242e+02, 2.1812635612487793e+02, 0., 0., 0., 1.,
       0.);
    cv::Mat P2 = (cv::Mat_<double>(3,4) <<
        4.9297293111444242e+02, 0., 3.2864122009277344e+02,
       -4.9681965181898590e+01, 0., 4.9297293111444242e+02,
       2.1812635612487793e+02, 0., 0., 0., 1., 0.);
    cv::Mat Q = (cv::Mat_<float>(4,4) << 
        1., 0., 0., -3.2864122009277344e+02, 0., 1., 0.,
       -2.1812635612487793e+02, 0., 0., 0., 4.9297293111444242e+02, 0.,
       0., 9.9225730968881827e+00, 0.);
    //double baseline = 1.0077095772657289e-01;
    //double focal = 4.9297293111444242e+02;
    cv::Mat cam0_map1, cam0_map2, cam1_map1, cam1_map2;
    cv::initUndistortRectifyMap(cam0, dist0, R1, P1, cv::Size2i(640,400), CV_32FC1, cam0_map1, cam0_map2);
    cv::initUndistortRectifyMap(cam1, dist1, R2, P2, cv::Size2i(640,400), CV_32FC1, cam1_map1, cam1_map2);
    cv::cuda::GpuMat g_cam0_map1(cam0_map1);
    cv::cuda::GpuMat g_cam0_map2(cam0_map2);
    cv::cuda::GpuMat g_cam1_map1(cam1_map1);
    cv::cuda::GpuMat g_cam1_map2(cam1_map2);

    cv::cuda::Stream cuda_stream1;
    //cv::cuda::Stream cuda_stream2(cudaStreamNonBlocking);
    //cv::Ptr<cv::cuda::StereoSGM> sgm = cv::cuda::createStereoSGM(0, 128, 10, 120, 15);
    //cv::Mat open_k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3));
    //cv::Ptr<cv::cuda::Filter> morph_filter = cv::cuda::createMorphologyFilter(cv::MORPH_OPEN, CV_32FC1, open_k);
    //cv::Ptr<cv::cuda::StereoBM> bm = cv::cuda::createStereoBM(64, 19);
    //bm->setUniquenessRatio(5);
    //bm->setTextureThreshold(10);
    //cv::Ptr<cv::cuda::DisparityBilateralFilter> filter = cv::cuda::createDisparityBilateralFilter(64);
    //cv::Ptr<cv::cuda::StereoConstantSpaceBP> csbp = cv::cuda::createStereoConstantSpaceBP(128,8,4,4,CV_16SC1);
    /*cv::cuda::GpuMat g_disp_map;
    cv::cuda::GpuMat g_disp_map_th;
    cv::cuda::GpuMat g_disp_map_scaled;
    cv::cuda::GpuMat g_disp_map_filtered;
    cv::cuda::GpuMat g_disp_map_dsz;
    cv::Mat disp_map;
    cv::Mat depth_img(400,640,CV_16UC1);*/

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
#if 0
    VPIStream vpi_stream;
    VPIImage vpi_frame_l_rect, vpi_frame_r_rect, vpi_disparity;
    CHECK_STATUS(vpiStreamCreate(0, &vpi_stream));
    VPIStereoDisparityEstimatorCreationParams vpi_stereoParams;
    CHECK_STATUS(vpiInitStereoDisparityEstimatorCreationParams(&vpi_stereoParams));
    //VPIConvertImageFormatParams vpi_convParams;
    //CHECK_STATUS(vpiInitConvertImageFormatParams(&vpi_convParams));
    //vpi_convParams.scale= 255.0 / (32 * vpi_stereoParams.maxDisparity);
    vpiImageCreate(640, 400, VPI_IMAGE_FORMAT_S16, 0, &vpi_disparity);
    //VPIImage vpi_disparity_scaled;
    //vpiImageCreate(640, 400, VPI_IMAGE_FORMAT_U8, 0, &vpi_disparity_scaled);
    VPIPayload vpi_stereo;
    CHECK_STATUS(vpiCreateStereoDisparityEstimator(VPI_BACKEND_CUDA, 640, 400, VPI_IMAGE_FORMAT_U8, &vpi_stereoParams, &vpi_stereo));
    VPIImageData vpi_data;
#endif
    double time = 0;
    struct timespec ts;    
    std::chrono::time_point<std::chrono::high_resolution_clock> t_a, t_b;
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
                //t_a = chrono::high_resolution_clock::now();
                //g_frame.upload(frame, cuda_stream1);
                g_frame_l = g_frame.colRange(g_frame.cols / 2, g_frame.cols);
                g_frame_r = g_frame.colRange(0, g_frame.cols / 2);
                cv::cuda::remap(g_frame_l, g_frame_l_rect, g_cam0_map1, g_cam0_map2, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar());
                cv::cuda::remap(g_frame_r, g_frame_r_rect, g_cam1_map1, g_cam1_map2, cv::INTER_LINEAR, cv::BORDER_REPLICATE, cv::Scalar());

                {
                    std::lock_guard<std::mutex> lk(depth_m);
                    img_ready = true;
                }
                depth_cv.notify_all();
                //t_b = chrono::high_resolution_clock::now();
                //cout<<"remap:"<< chrono::duration_cast<chrono::milliseconds>(t_b-t_a).count()<<"ms"<<endl;
                //t_a = chrono::high_resolution_clock::now();
                estimator.inputImage(time, g_frame_l_rect, g_frame_r_rect, cuda_stream1);
                //t_b = chrono::high_resolution_clock::now();
                //cout<<"tracker:"<< chrono::duration_cast<chrono::milliseconds>(t_b-t_a).count()<<"ms"<<endl;
#if 0
                sgm->compute(g_frame_l_rect, g_frame_r_rect, g_disp_map, cuda_stream1);
                cv::cuda::resize(g_disp_map, g_disp_map_dsz, cv::Size(320,200), 0, 0, cv::INTER_LINEAR, cuda_stream1);
                cv::cuda::threshold(g_disp_map_dsz, g_disp_map_th, 0, 0, cv::THRESH_TOZERO, cuda_stream1); //sgm have negative disp
                g_disp_map_th.convertTo(g_disp_map_scaled, CV_32FC1, 1.0/16.0, cuda_stream1); //sgm have 4 fractional bits
                morph_filter->apply(g_disp_map_scaled, g_disp_map_filtered, cuda_stream1);
                g_disp_map_filtered.download(disp_map);
                //cuda_stream1.waitForCompletion();
                cv::divide(baseline*focal*1000, disp_map, depth_img, CV_16UC1); //cuda::divide work different
                depth_img_pub.publish(cv_bridge::CvImage(ros_header, "mono16", depth_img).toImageMsg());
#endif
                //t_b = chrono::high_resolution_clock::now();
                //cout<<"total:"<< chrono::duration_cast<chrono::milliseconds>(t_b-t_a).count()<<"ms"<<endl;
#if 0
//                cv::cuda::remap(g_frame_l, g_frame_l_rect, g_cam0_map1, g_cam0_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
//                cv::cuda::remap(g_frame_r, g_frame_r_rect, g_cam1_map1, g_cam1_map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
                //t_a = chrono::high_resolution_clock::now();
                g_frame_l_rect.download(frame_l_rect);
                g_frame_r_rect.download(frame_r_rect);
                vpiImageCreateWrapperOpenCVMat(frame_l_rect, 0, &vpi_frame_l_rect); 
                vpiImageCreateWrapperOpenCVMat(frame_r_rect, 0, &vpi_frame_r_rect);
                vpiSubmitStereoDisparityEstimator(vpi_stream, VPI_BACKEND_CUDA, vpi_stereo, vpi_frame_l_rect, vpi_frame_r_rect, vpi_disparity, NULL, NULL);
                //vpiSubmitConvertImageFormat(vpi_stream, VPI_BACKEND_CUDA, vpi_disparity, vpi_disparity_scaled, &vpi_convParams);
                vpiStreamSync(vpi_stream);
                vpiImageLockData(vpi_disparity, VPI_LOCK_READ, VPI_IMAGE_BUFFER_HOST_PITCH_LINEAR, &vpi_data);
                vpiImageDataExportOpenCVMat(vpi_data, &disp_map);
                cv::divide(baseline*focal*32*1000, disp_map, depth_img, CV_16UC1); //Disparities are in Q10.5 format, so to map it to float, it gets divided by 32
                depth_img_pub.publish(cv_bridge::CvImage(ros_header, "mono16", depth_img).toImageMsg());
                vpiImageUnlock(vpi_disparity);
                //t_b = chrono::high_resolution_clock::now();
                //cout<<"depth:"<< chrono::duration_cast<chrono::milliseconds>(t_b-t_a).count()<<"ms"<<endl;
#endif
#if 0
                cc++;
                if (cc>=4)  {
                    //cout<<"depth\n";
                    cc=0;
                    working=true;
                    ros_header.stamp.sec = ts.tv_sec;
                    ros_header.stamp.nsec = ts.tv_nsec;
                    //t_a = chrono::high_resolution_clock::now();
                    g_frame_l_rect.copyTo(g_frame_l_buf, cuda_stream2);
                    g_frame_r_rect.copyTo(g_frame_r_buf, cuda_stream2);
                    bm->compute(g_frame_l_buf, g_frame_r_buf, g_disp_map, cuda_stream2);
                    filter->apply(g_disp_map, g_frame_l_buf, g_disp_map_filtered, cuda_stream2);
                    cv::cuda::reprojectImageTo3D(g_disp_map_filtered, g_3d_img, Q, 3, cuda_stream2);
                    //t_b = chrono::high_resolution_clock::now();
                    //cout<<"disp:"<< chrono::duration_cast<chrono::milliseconds>(t_b-t_a).count()<<"ms"<<endl;
                }
                //cv::cuda::divide(cv::Scalar(baseline * focal), g_disp_map, g_depth_img, 1, CV_32FC1);
                //g_depth_img.convertTo(g_depth_img_scaled, CV_16UC1, 1000);
                //filter->apply(g_disp_map, g_frame_l_rect, g_disp_map_filtered);
                //cv::cuda::reprojectImageTo3D(g_disp_map, g_3d_img, Q, 3);
                //cv::cuda::split(g_3d_img, g_split_img);
                //cv::cuda::threshold(g_split_img[2], g_split_img[1], 0, 0, cv::THRESH_TOZERO); //reuse unused
                //cv::cuda::threshold(g_split_img[1], g_split_img[0], 6 , 0, cv::THRESH_TRUNC);
                //g_split_img[0].convertTo(g_depth_img_scaled, CV_16UC1, 1000);
                //g_depth_img_scaled.download(depth_img);
                //auto t_b = chrono::high_resolution_clock::now();
                //cout<<"depth:"<< chrono::duration_cast<chrono::milliseconds>(t_b-t_a).count()<<"ms"<<endl;

                //std_msgs::Header header;
                //header.frame_id="image";
                //header.stamp=ros::Time::now();
                //depth_img_pub.publish(cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_32FC3, img_3d).toImageMsg());
                if (working && cuda_stream2.queryIfComplete()) {
                    working=false;
                    cv::cuda::split(g_3d_img, g_split_img);
                    cv::cuda::threshold(g_split_img[2], g_split_img[0], 10, 0, cv::THRESH_TRUNC); //reuse unused
                    g_split_img[0].convertTo(g_depth_img_scaled, CV_16UC1, 1000);
                    g_depth_img_scaled.download(depth_img);
                    //ros_header.stamp.sec = ts.tv_sec;
                    //ros_header.stamp.nsec = ts.tv_nsec;
                    depth_img_pub.publish(cv_bridge::CvImage(ros_header, "mono16", depth_img).toImageMsg());
                }
#endif
            }
        }
    }

    cudaFree(&unified_ptr);

    //vpiStreamDestroy(vpi_stream);
    //vpiImageDestroy(vpi_frame_l_rect);
    //vpiImageDestroy(vpi_frame_r_rect);
    //vpiImageDestroy(vpi_disparity);
    //vpiImageDestroy(vpi_disparity_scaled);
    //vpiPayloadDestroy(vpi_stereo);

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

    std::thread sync_thread(sync_process);
    std::thread depth_thread(depth_process);

    ros::init(argc, argv, "vins_estimator");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);

    depth_img_pub = n.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 500, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_restart = n.subscribe("/vins_restart", 1, restart_callback);
    //ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    //ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);
    ros::Subscriber sub_send_apm = n.subscribe("/vins_send_pose_apm", 1, send_apm_callback);

    ros::spin();

    printf("game over\n");
    gogogo = false;
    estimator.gogogo = false;
    sync_thread.join();
    depth_thread.join();

    return 0;
}
