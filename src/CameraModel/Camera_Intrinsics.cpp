/*
 * Camera_Intrinsics.cpp
 *
 *  Created on: Jul 7, 2018
 *      Author: zer0
 */
#include "CameraModel/Camera_Intrinsics.h"

using namespace ImageProcess;

CameraIntrinsics::CameraIntrinsics(double fx, double fy, double cx, double cy)
{
	M_ << fx, 0, cx,
			0, fy, cy,
			0, 0, 1;
	M_inv_ = M_.inverse();
}

CameraIntrinsics::~CameraIntrinsics()
{}

/// 设置相机参数
bool CameraIntrinsics::SetCameraParams(const Vec& params)
{
	if(params.size() != 4)
	{
		return false;
	}
	M_ << params[0], 0, params[2],
			0, params[1], params[3],
			0, 0, 1;
	return true;
}

/// 相机坐标点转换到图像坐标点
Vec2 CameraIntrinsics::Camera2Image(const Vec2& cam_pt)
{
	Vec2 img_pt = Vec3(M_ * cam_pt.homogeneous()).hnormalized();
	return img_pt;
}
/// 图像坐标点转换到相机坐标点
Vec2 CameraIntrinsics::Image2Camera(const Vec2& img_pt)
{
	Vec2 cam_pt = Vec3(M_.inverse()*img_pt.homogeneous()).hnormalized();
	return cam_pt;
}
