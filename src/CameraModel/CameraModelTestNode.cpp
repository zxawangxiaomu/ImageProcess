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
#include "CameraModel/Camera_Intrinsics_Brown.h"

using namespace ImageProcess;

bool Img2Cam(const std::shared_ptr<CameraIntrinsics> cam_intrin_ptr, const std::vector<double>& params,
        const Vec2& img_pt, const Vec2& expected_cam_pt, std::string& err_str)
{
    bool ret = cam_intrin_ptr->setCameraParams(params);
    if (!ret)
    {
        err_str = "Failed to SetCameraParams\n";
        return false;
    }

    Vec2 cam_pt = cam_intrin_ptr->image2Camera(img_pt);
    if ((cam_pt - expected_cam_pt).norm() > 1e-6)
    {
        err_str = "Failed to Image2Camera\n";
        return false;
    }

    return true;
}

bool Cam2Img(const std::shared_ptr<CameraIntrinsics> cam_intrin_ptr, const std::vector<double>& params,
        const Vec2& cam_pt, const Vec2& expected_img_pt, std::string& err_str)
{
    bool ret = cam_intrin_ptr->setCameraParams(params);
    if (!ret)
    {
        err_str = "Failed to SetCameraParams\n";
        return false;
    }

    Vec2 img_pt = cam_intrin_ptr->camera2Image(cam_pt);
    if ((img_pt - expected_img_pt).norm() > 1e-6)
    {
        err_str = "Failed to Image2Camera\n";
        return false;
    }

    return true;
}

TEST(TransformCoordinate, Img2Cam)
{
    std::shared_ptr<CameraIntrinsics> cam_intrin_ptr = std::make_shared<
            CameraIntrinsics>();
    std::vector<double> params = { 963.777527, 963.777527, 1280, 800 };
    Vec2 img_pt, cam_pt;
    img_pt << 120, 756;
    cam_pt[0] = (img_pt[0] - params[2]) / params[0];
    cam_pt[1] = (img_pt[1] - params[3]) / params[1];
    std::string err_str;
    ASSERT_TRUE(Img2Cam(cam_intrin_ptr, params, img_pt, cam_pt, err_str))<< err_str;
}

TEST(TransformCoordinate, Cam2Img)
{
    std::shared_ptr<CameraIntrinsics> cam_intrin_ptr = std::make_shared<CameraIntrinsics>();
    std::vector<double> params =
    { 956, 956, 1280, 800 };
    Vec2 img_pt, cam_pt;
    img_pt << 120, 756;
    cam_pt[0] = (img_pt[0] - params[2]) / params[0];
    cam_pt[1] = (img_pt[1] - params[3]) / params[1];
    std::string err_str;
    ASSERT_TRUE(Cam2Img(cam_intrin_ptr, params, cam_pt, img_pt, err_str))<< err_str;
}

TEST(Distortion, AddAndRemove)
{
    std::shared_ptr<CameraIntrinsics> cam_intrin_ptr = std::make_shared<CameraIntrinsicsBrown>();
    std::vector<double> params ={ 963.777527, 963.777527, 632.300476, 398.730255,
                                  0.002593, 0.054237, -0.179120, 0.000777, 0.000321 };
    cam_intrin_ptr->setCameraParams(params);
    Vec2 ud_img_pt = { 120, 756 };
    Vec2 ud_cam_pt = cam_intrin_ptr->image2Camera(ud_img_pt);
    Vec2 cam_pt = cam_intrin_ptr->addDistortion(ud_cam_pt);
    ud_cam_pt = cam_intrin_ptr->removeDistortion(cam_pt);
    Vec2 ud_img_pt_temp = cam_intrin_ptr->camera2Image(ud_cam_pt);
    double err = (ud_img_pt_temp - ud_img_pt).norm();
    ASSERT_TRUE(err < 1e-6) << "err = " << err;
}

int main(int argc, char** argv)
{
std::cout << "running test" << std::endl;
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
