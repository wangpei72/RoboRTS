//
// Created by wang_shuai on 19-12-27.
//

#include "realsense.h"
#include <sensor_msgs/PointCloud.h>

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
    std::vector<cv::Point3f> RS_Driver::DepthPixel2World(cv::Mat &img) {
        cv::Mat img_out;
        img_out.create(img.size(),img.type());
        //roborts_camera::CameraInfo cameraInfo;
        float intrinsicL[3][3] = {{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
                                  {0.,                   9.19093433238763e+02, 3.70129208057065e+02},
                                  {0.,                   0.,                   1},
        };
        float intrinsicR[3][3] = {{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
                                  {0.,                   9.19093433238763e+02, 3.70129208057065e+02},
                                  {0.,                   0.,                   1},
        };

        //realsense and MVS extrinsic mat info

        //realsense and MVS extrinsic mat info
        float extrinsic[3][4] = {{9.9930980513982504e-01,  3.5070411883416031e-02,
                                         1.2246614296712125e-02, -2.3640906380346682e-01},
                                 {-3.6161562130512234e-02, 9.9384880431654243e-01,
                                         1.0467519086612581e-01, -2.6589124679310618e+00},
                                 {-8.5002809180629770e-03, -1.0504580129118116e-01,
                                         9.9443105585827696e-01, 3.4991621301756459e-01},
        };
        float rotation[3][3] = {
                {9.9930980513982504e-01,  3.5070411883416031e-02,
                        1.2246614296712125e-02},
                {-3.6161562130512234e-02, 9.9384880431654243e-01,
                        1.0467519086612581e-01},
                {-8.5002809180629770e-03, -1.0504580129118116e-01,
                        9.9443105585827696e-01},
        };
        float translation[3] = {
                 -2.3640906380346682e-01,
                -2.6589124679310618e+00,
                 3.4991621301756459e-01,
        };
        //depth camera intrinsicL mat
        cv::Mat LR = cv::Mat(3, 3, CV_32F, intrinsicL);
        cv::Mat RR = cv::Mat(3,3,CV_32F,intrinsicR);
        //depth camera rotation and translation mat
        cv::Mat M = cv::Mat(3, 4, CV_32F, extrinsic);
        //rotation mat and translation mat
        cv::Mat R = cv::Mat(3,3,CV_32F,rotation);
        cv::Mat T = cv::Mat(3,1,CV_32F,translation);
        //world coordinate mat
        cv::Mat XYZ = cv::Mat(3, 1,CV_32F);
        cv::Mat uv = cv::Mat(3,1,CV_32F);
        //world point cloud
        std::vector<cv::Point3f> world_points;
        std::vector<cv::Point3f> pixel_points;
        //  XYZ in world
        for (int i = 0; i < img_out.rows; ++i) {
            for (int j = 0; j < img_out.cols; ++j) {
                cv::Point3f point3f;
                cv::Point3f pointPixelWithDepth;
                point3f.x=0;
                point3f.y=0;
                point3f.z=0;
                uv.at<float>(0,0)=j;
                uv.at<float>(1,0)=i;
                uv.at<float>(2,0)=1;
                float z = img.at<float>(i, j);
                cv::Mat res=z*LR.inv()*uv;
                //geometric trans between snesors
                res =R*res +T;
                point3f.x=res.at<float>(0, 0);
                point3f.y=res.at<float>(1, 0);
                point3f.z=z;
                //uv here stand for transformed
                uv=RR*res/z;
                pointPixelWithDepth.x=uv.at<float>(0, 0);
                pointPixelWithDepth.y=uv.at<float>(1, 0);
                pointPixelWithDepth.z=z;
                //uv should be in the range of resolution
                if(pointPixelWithDepth.x > 0 && pointPixelWithDepth.y > 0
                   && pointPixelWithDepth.y < 2048 && pointPixelWithDepth.x < 3072){
                    pixel_points.push_back(pointPixelWithDepth);
                }
                world_points.push_back(point3f);
            }
        }
        float ratio=pixel_points.size()/(2048*3072);
        return pixel_points;
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