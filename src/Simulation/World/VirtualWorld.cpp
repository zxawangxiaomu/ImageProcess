//
// Created by zer0 on 19-2-2.
//

#include "VirtualWorld.h"

ImageProcess::VirtualWorld::VirtualWorld()
{
    // 初始化3d点云
    // 初始化相机模型
}

ImageProcess::VirtualWorld::~VirtualWorld() {

}

void ImageProcess::VirtualWorld::InitPCL()
{
    // 初始化为一个立方体
    int index = 0;
    for(int i = 0; i < 100; ++i)
    {
        for(int j = 0; j < 100; ++j)
        {
            for(int k = 0; k < 100; ++k)
            {
                CodePoint3d code_point3d;
                code_point3d.idx = index;
                code_point3d.point << i, j, k;
            }
        }
    }
}

void ImageProcess::VirtualWorld::InitCameras()
{
    cameras_.clear();
    // 初始化相机内参
    Camera camera;
    std::shared_ptr<CameraIntrinsics> camera_intrin = std::make_shared<CameraIntrinsicsBrown>();
    std::vector<double> params = {1000, 1003, 640, 480, 0, 0, 0, 0, 0};
    camera_intrin->setCameraParams(params);
    // 初始化相机的运动(注意，初始位姿设置为单位阵)
    std::map<TimePoint, Mat4> motions;
    TimePoint current_time = Now();
    Mat4 motion = Mat4::Identity();
    motions[current_time] = motion;
    current_time = Now();
    motion.topLeftCorner(3, 3)  = Eigen::AngleAxisd(30*M_PI/180, Eigen::Vector3d::UnitZ()) * motion.topLeftCorner(3, 3);



}
