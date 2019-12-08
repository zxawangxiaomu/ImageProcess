//
// Created by zer0 on 19-2-2.
//

#ifndef IMAGEPROCESS_VIRTUALWORLD_H
#define IMAGEPROCESS_VIRTUALWORLD_H

#include <Eigen/Dense>
#include "Numeric/eigen_alias_definition.hpp"
#include "CameraModel/Camera_Intrinsics_Brown.h"
#include "System/Time/sys_time.hpp"
#include <thread>

namespace ImageProcess
{

typedef Vec3 Point3d;

struct CodePoint3d
{
    int idx;
    Point3d point;
};

struct Camera
{
    std::shared_ptr<CameraIntrinsics> camera_intrin;
    std::map<TimePoint, Mat4> motions;
};

class VirtualWorld
{
public:
    VirtualWorld();
    virtual ~VirtualWorld();

private:
    void InitPCL();
    void InitCameras();
private:
    std::vector<Point3d> pcl_;
    CodePoint3d code_pcl;

    std::vector<Camera> cameras_;
};

}

#endif //IMAGEPROCESS_VIRTUALWORLD_H
