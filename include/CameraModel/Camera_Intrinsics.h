/*
 * CameraIntrinsics.h
 *
 *  Created on: Jul 7, 2018
 *      Author: zer0
 */

#ifndef INCLUDE_CAMERAMODEL_CAMERA_INTRINSICS_H_
#define INCLUDE_CAMERAMODEL_CAMERA_INTRINSICS_H_

#include "Numeric/eigen_alias_definition.hpp"

namespace ImageProcess
{
class CameraIntrinsics
{
public:
    CameraIntrinsics(double fx = 0.0, double fy = 0.0, double cx = 0.0, double cy = 0.0);
    virtual ~CameraIntrinsics();

    /// 设置相机参数
    virtual bool setCameraParams(const std::vector<double>& params);
    /// 相机坐标点转换到图像坐标点
    virtual Vec2 camera2Image(const Vec2& cam_pt);
    /// 图像坐标点转换到相机坐标点
    virtual Vec2 image2Camera(const Vec2& img_pt);

    // 加畸变
    virtual Vec2 addDistortion(const Vec2& ud_cam_pt);
    // 去畸变
    virtual Vec2 removeDistortion(const Vec2& cam_pt);
protected:
    Mat3 M_;
    Mat3 M_inv_;
};
}

#endif /* INCLUDE_CAMERAMODEL_CAMERA_INTRINSICS_H_ */
