//
// Created by wang_shuai on 19-12-27.
//

#include "realsense.h"

namespace roborts_camera
{
    RS_Driver::RS_Driver(roborts_camera::CameraInfo cameraInfo_) : CameraBase(cameraInfo_),align_to_color(RS2_STREAM_COLOR)
    {
        rs2::context context;
        ROS_INFO("realsnese device number : %d",context.query_devices().size());

        if( context.query_devices().size() == 0)
        {
            ROS_ERROR("han pi , restart the realsense device ,plz");
            exit(-1);
        }
        device_ = context.query_devices()[0];

        config_.enable_device(device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        config_.enable_stream(RS2_STREAM_COLOR,
                cameraInfo_.resolution_width,
                cameraInfo_.resolution_height,
                RS2_FORMAT_ANY,
                60);
        config_.enable_stream(RS2_STREAM_DEPTH,
                cameraInfo_.depth_resolution_width,
                cameraInfo_.depth_resolution_height,
                RS2_FORMAT_ANY,
                60);
        sensors_ = device_.query_sensors();
        ROS_INFO("realsense sensors number :%d",sensors_.size());

        for( auto s : sensors_)
        {
            ROS_INFO("%s",s.get_info(RS2_CAMERA_INFO_NAME));
        }
        sensor_controls("RGB Camera",RS2_OPTION_EXPOSURE, 10);

        pipeline_.start(config_);




        camera_initialized_ = 1;

    }

    void RS_Driver::StartReadCamera(cv::Mat &img, cv::Mat &depth)
    {

        img.create(camera_info_.resolution_height,
                   camera_info_.resolution_width,
                   CV_8UC3);

        depth.create(camera_info_.resolution_height,
                      camera_info_.resolution_width,
                      CV_16UC1);
        rs2::frameset frameset = pipeline_.wait_for_frames();


        frameset = align_to_color.process(frameset);
        rs2::frame depth_frame = frameset.get_depth_frame();
        rs2::frame color_frame = frameset.get_color_frame();

        //rs2::video_frame vf = color_frame.as<rs2::video_frame>();

        //ROS_INFO("%d %d",vf.get_width(),vf.get_height());

        //free(img.data);// 简直有毒

        //img.data = (unsigned char *)malloc(sizeof(unsigned char) * 1280*720*10);

        img.data = (uchar *)color_frame.get_data();
        depth.data = (uchar*)depth_frame.get_data();

        //ROS_INFO("%f",depth.at<ushort>(640,360));
    }

    void RS_Driver::StartReadDepth(cv::Mat &img)
    {
        rs2::frameset frameset = pipeline_.wait_for_frames(2000);

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

    /*void RS_Driver::sensor_controls(std::string sensor_name, rs2_option option, int value)
    {
        for( auto sensor_ : sensors_)
        {
            if(sensor_.get_info(RS2_CAMERA_INFO_NAME) == sensor_name)
            {
                sensor_.set_option(option,value);
            }
        }
    }*/

   /* void RS_Driver::sensor_controls(std::string sensor_name, rs2_option option, bool value)
    {
         for( auto sensor_ : sensors_)
        {
            if(sensor_.get_info(RS2_CAMERA_INFO_NAME) == sensor_name)
            {

                sensor_.set_option(option,value);
            }
        }
    }*/

    void RS_Driver::sensor_controls(std::string sensor_name, rs2_option option, float value)
    {
         for( auto sensor_ : sensors_)
        {
            if(sensor_.get_info(RS2_CAMERA_INFO_NAME) == sensor_name)
            {
                sensor_.set_option(option,value);
                ROS_INFO("success");
            }
        }
    }
}