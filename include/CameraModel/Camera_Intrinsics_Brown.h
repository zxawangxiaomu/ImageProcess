/*
 * Camera_Intrinsics_Brown.h
 *
 *  Created on: Jul 8, 2018
 *      Author: zer0
 */

#ifndef INCLUDE_CAMERAMODEL_CAMERA_INTRINSICS_BROWN_H_
#define INCLUDE_CAMERAMODEL_CAMERA_INTRINSICS_BROWN_H_

#include "CameraModel/Camera_Intrinsics.h"
#include <vector>

namespace ImageProcess
{
class CameraIntrinsicsBrown : public CameraIntrinsics
{
public:
    CameraIntrinsicsBrown(double fx = 0, double fy = 0, double cx = 0, double cy = 0,
    		double k1 = 0, double k2 = 0, double k3 = 3, double t1 = 0, double t2 = 0);
    virtual ~CameraIntrinsicsBrown();

    virtual bool setCameraParams(const std::vector<double>& params) override;

    virtual Vec2 addDistortion(const Vec2& ud_cam_pt) override;
    virtual Vec2 removeDistortion(const Vec2& cam_pt) override;
private:
    static Vec2 distortionFunction(const std::vector<double>& k, const Vec2& ud_cam_pt);
    std::vector<double> k_;
};
}



#endif /* INCLUDE_CAMERAMODEL_CAMERA_INTRINSICS_BROWN_H_ */
