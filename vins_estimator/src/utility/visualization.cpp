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

static struct sockaddr_un chobits_addr, chobits_local_addr;
static int chobits_sock;
extern int pub_sock;
extern struct sockaddr_in pub_addr;

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

        if (pub_addr.sin_family == AF_INET) {
            double odo_msg[] = {0, px, py, pz, qx, qy, qz, qw, vx, vy, vz};
            if (sendto(pub_sock, odo_msg, sizeof(odo_msg), 0, (struct sockaddr*)&pub_addr, sizeof(pub_addr)) < 0) {
                perror("sendto failed");
            }
        }
    }

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
}
