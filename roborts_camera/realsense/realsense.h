//
// Created by wang_shuai on 19-12-27.
//

#ifndef ROBORTS_CAMERA_REALSENSE_H
#define ROBORTS_CAMERA_REALSENSE_H

#include <librealsense2/rs.hpp>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "../camera_param.h"
#include "../camera_base.h"
#include "alg_factory/algorithm_factory.h"


namespace  roborts_camera{
    class RS_Driver : public CameraBase
    {
    public:
        explicit RS_Driver(CameraInfo cameraInfo_);

        void StartReadCamera(cv::Mat &img, cv::Mat &depth) override;

        void StartReadDepth(cv::Mat &img);

        ~RS_Driver() override;

        //virtual void StartReadMotion();

        void StopReadCamera() override;
    private:
        rs2::device device_;

        rs2::config config_;

        rs2::pipeline pipeline_;

        rs2::align align_to_color;

    };
    roborts_common::REGISTER_ALGORITHM(CameraBase, "realsense", RS_Driver, CameraInfo);
}





#endif //ROBORTS_CAMERA_REALSENSE_H
