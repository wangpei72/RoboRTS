//
// Created by wang_shuai on 19-12-27.
//

#include "realsense.h"

namespace roborts_camera
{
    RS_Driver::RS_Driver(roborts_camera::CameraInfo cameraInfo_) : CameraBase(cameraInfo_)
    {
        rs2::context context;

        device_ = context.query_devices()[0];

        config_.enable_device(device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        pipeline_.start(config_);

        camera_initialized_ = 1;

    }

    void RS_Driver::StartReadCamera(cv::Mat &img)
    {
        rs2::frameset frameset = pipeline_.wait_for_frames(200);

        rs2::frame color_frame = frameset.get_color_frame();

        img.data = (uchar *)color_frame.get_data();
    }

    void RS_Driver::StartReadDepth(cv::Mat &img)
    {
        rs2::frameset frameset = pipeline_.wait_for_frames(200);

        rs2::frame color_frame = frameset.get_depth_frame();

        img.data = (uchar *)color_frame.get_data();

        cv::cvtColor(img,img,CV_RGB2BGR);
    }

    void RS_Driver::StopReadCamera()
    {
        pipeline_.stop();
    }

    RS_Driver::~RS_Driver()
    {

    }
}