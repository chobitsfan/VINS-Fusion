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

using namespace Eigen;

struct sockaddr_un chobits_addr, chobits_local_addr;
static int chobits_sock;

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

        Quaterniond tmp_Q = Quaterniond(estimator.Rs[WINDOW_SIZE]);

        float chobits_msg[10] = { (float)tmp_Q.w(), (float)tmp_Q.x(), (float)tmp_Q.y(), (float)tmp_Q.z(), (float)px, (float)py, (float)pz, (float)vx, (float)vy, (float)vz };
        sendto(chobits_sock, chobits_msg, sizeof(chobits_msg), 0, (struct sockaddr*)&chobits_addr, sizeof(chobits_addr));
    }
}
