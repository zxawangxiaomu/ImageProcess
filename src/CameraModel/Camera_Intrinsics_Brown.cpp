/*
 * Camera_Intrinsics_Brown.cpp
 *
 *  Created on: Jul 8, 2018
 *      Author: zer0
 */
#include "CameraModel/Camera_Intrinsics_Brown.h"

using namespace ImageProcess;

CameraIntrinsicsBrown::CameraIntrinsicsBrown(double fx, double fy, double cx,
        double cy, double k1, double k2, double k3, double t1, double t2)
        : CameraIntrinsics(fx, fy, cx, cy)
{
    k_.resize(5, 0);
    k_[0] = k1;
    k_[1] = k2;
    k_[3] = k3;
    k_[4] = t1;
    k_[5] = t2;
}

CameraIntrinsicsBrown::~CameraIntrinsicsBrown()
{
}

bool CameraIntrinsicsBrown::setCameraParams(const std::vector<double>& params)
{
    // 4 + 5
    if (params.size() != 9)
    {
        return false;
    }
    // 内参
    M_ << params[0], 0, params[2], 0, params[1], params[3], 0, 0, 1;
    // 畸变
    k_[0] = params[4];
    k_[1] = params[5];
    k_[2] = params[6];
    k_[3] = params[7];
    k_[4] = params[8];

    return true;
}

Vec2 CameraIntrinsicsBrown::addDistortion(const Vec2& ud_cam_pt)
{
    Vec2 cam_pt = ud_cam_pt + distortionFunction(k_, ud_cam_pt);
    return cam_pt;
}

Vec2 CameraIntrinsicsBrown::removeDistortion(const Vec2& cam_pt)
{
    const double epsilon = 1e-10; // 收敛条件
    Vec2 ud_cam_pt = cam_pt;
    Vec2 distortion = distortionFunction(k_, ud_cam_pt);
    while ((ud_cam_pt + distortion - cam_pt).lpNorm<1>() > 1e-10) // 1范数
    {
        ud_cam_pt = cam_pt - distortion;
        distortion = distortionFunction(k_, ud_cam_pt);
    }
    return ud_cam_pt;
}

Vec2 CameraIntrinsicsBrown::distortionFunction(const std::vector<double>& k,
        const Vec2& ud_cam_pt)
{
    double k1 = k[0], k2 = k[1], k3 = k[2], t1 = k[3], t2 = k[4];
    double x = ud_cam_pt[0], y = ud_cam_pt[1];
    double r2 = x * x + y * y;
    double r4 = r2 * r2;
    double r6 = r4 * r2;

    // 径向畸变
    double k_diff = k1 * r2 + k2 * r4 + k3 * r6;
    // 切向畸变
    double t_x = t2 * (r2 + 2 * x * x) + 2 * t1 * x * y;
    double t_y = t1 * (r2 + 2 * y * y) + 2 * t2 * x * y;

    return
    {   x*k_diff + t_x, y*k_diff + t_y};
}

