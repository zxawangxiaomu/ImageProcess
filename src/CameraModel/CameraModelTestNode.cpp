/*
 * CamerModelTestNode.cpp
 *
 *  Created on: Jul 7, 2018
 *      Author: zer0
 */
#include <gtest/gtest.h>
#include <memory>
#include <numeric>
#include <string>
#include <iostream>
#include "CameraModel/Camera_Intrinsics.h"

using namespace ImageProcess;

bool Img2Cam(const std::shared_ptr<CameraIntrinsics> cam_intrin_ptr, const Vec& params,
		const Vec2& img_pt, const Vec2& expected_cam_pt, std::string& err_str)
{
	bool ret = cam_intrin_ptr->SetCameraParams(params);
	if (!ret)
	{
		err_str = "Failed to SetCameraParams\n";
		return false;
	}

	Vec2 cam_pt = cam_intrin_ptr->Image2Camera(img_pt);
	if((cam_pt - expected_cam_pt).norm() > 1e-6)
	{
		err_str = "Failed to Image2Camera\n";
		return false;
	}

	return true;
}

bool Cam2Img(const std::shared_ptr<CameraIntrinsics> cam_intrin_ptr, const Vec& params,
		const Vec2& cam_pt, const Vec2& expected_img_pt, std::string& err_str)
{
	bool ret = cam_intrin_ptr->SetCameraParams(params);
	if (!ret)
	{
		err_str = "Failed to SetCameraParams\n";
		return false;
	}

	Vec2 img_pt = cam_intrin_ptr->Camera2Image(cam_pt);
	if((img_pt - expected_img_pt).norm() > 1e-6)
	{
		err_str = "Failed to Image2Camera\n";
		return false;
	}

	return true;
}

TEST(TransformCoordinate, Img2Cam)
{
	std::shared_ptr<CameraIntrinsics> cam_intrin_ptr = std::make_shared<CameraIntrinsics>();
	Vec params(4);
	params << 956, 956, 1280, 800;
	Vec2 img_pt, cam_pt;
	img_pt << 120, 756;
	cam_pt[0] = (img_pt[0] - params[2])/params[0];
	cam_pt[1] = (img_pt[1] - params[3])/params[1];
	std::string err_str;
	ASSERT_TRUE(Img2Cam(cam_intrin_ptr, params, img_pt, cam_pt, err_str)) << err_str;

}

TEST(TransformCoordinate, Cam2Img)
{
	std::shared_ptr<CameraIntrinsics> cam_intrin_ptr = std::make_shared<CameraIntrinsics>();
	Vec params(4);
	params << 956, 956, 1280, 800;
	Vec2 img_pt, cam_pt;
	img_pt << 120, 756;
	cam_pt[0] = (img_pt[0] - params[2])/params[0];
	cam_pt[1] = (img_pt[1] - params[3])/params[1];
	std::string err_str;
	ASSERT_TRUE(Cam2Img(cam_intrin_ptr, params, cam_pt, img_pt, err_str)) << err_str;
}


int main(int argc, char** argv)
{
	std::cout << "running test" << std::endl;
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
