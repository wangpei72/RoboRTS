//
// Created by wang_shuai on 19-12-27.
//

#ifndef ROBORTS_CAMERA_MVS_DRIVER_H
#define ROBORTS_CAMERA_MVS_DRIVER_H

#include "../camera_base.h"
#include "../camera_param.h"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "MvCameraControl.h"
#include "alg_factory/algorithm_factory.h"

namespace roborts_camera{
    class MVS_Driver : public CameraBase
    {
    public:
        explicit MVS_Driver(CameraInfo cameraInfo_);

        void StartReadCamera( cv::Mat &img,cv::Mat &depth) override;

        void StopReadCamera();

        void SetEnumValue(std::string name, int value);

        void WriteParam() ;

        void LoadParam() ;

        ~MVS_Driver() override;
    private:

        void* handle = nullptr;

        const int maxDataSize =  1024*1024*40 ;

    };
    roborts_common::REGISTER_ALGORITHM(CameraBase, "mvs", MVS_Driver, CameraInfo);
}


#endif //ROBORTS_CAMERA_MVS_DRIVER_H
