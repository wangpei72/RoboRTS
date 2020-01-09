//
// Created by wang_shuai on 19-12-27.
//

#include "realsense.h"
#include <sensor_msgs/PointCloud.h>

namespace roborts_camera
{
    RS_Driver::RS_Driver(roborts_camera::CameraInfo cameraInfo_) : CameraBase(cameraInfo_)
    {
        rs2::context context;

        device_ = context.query_devices()[0];

        //ROS_INFO("%d",context.query_devices().size());

        config_.enable_device(device_.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

        pipeline_.start(config_);

        camera_initialized_ = 1;

    }

    void RS_Driver::StartReadCamera(cv::Mat &img)
    {

        img.create(camera_info_.resolution_height,
                   camera_info_.resolution_width,
                   CV_16UC1);
        rs2::frameset frameset = pipeline_.wait_for_frames();

        rs2::colorizer color;

        rs2::frame color_frame = frameset.get_depth_frame();


        //rs2::video_frame vf = color_frame.as<rs2::video_frame>();

        //ROS_INFO("%d,%d",color_frame.);

        //free(img.data);// 简直有毒

        //img.data = (unsigned char *)malloc(sizeof(unsigned char) * 1280*720*10);

        img.data = (uchar *)color_frame.get_data();
    }
    void RS_Driver::DepthPixel2Wolrd(cv::Mat &img) {
        cv::Mat img_out;
        img.copyTo(img_out);
        //roborts_camera::CameraInfo cameraInfo;
        float intrinsic[3][3] = {{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
                                 {0.,                   9.19093433238763e+02, 3.70129208057065e+02},
                                 {0.,                   0.,                   1},
        };
        //realsense and MVS extrinsic mat info
        float extrinsic[3][4] = {{9.9930980513982504e-01,  3.5070411883416031e-02,
                                         1.2246614296712125e-02, -2.3640906380346682e-01},
                                 {-3.6161562130512234e-02, 9.9384880431654243e-01,
                                         1.0467519086612581e-01, -2.6589124679310618e+00},
                                 {-8.5002809180629770e-03, -1.0504580129118116e-01,
                                         9.9443105585827696e-01, 3.4991621301756459e-01},
        };
        //depth camera intrinsic mat
        cv::Mat LR = cv::Mat(3, 3, CV_32F, intrinsic);
        //depth camera rotation and translation mat
        cv::Mat LRT = cv::Mat(4, 4, CV_32F, extrinsic);
        //world coordinate mat
        cv::Mat XYZ = cv::Mat(3, 1, CV_32F);

        /*//calculate M mat
        cv::Mat M = LR*LRT;*/
        std::vector<cv::Point3f> world_points;

        //  XYZ in world
        for (int i = 0; i < img_out.rows; ++i) {
            for (int j = 0; j < img_out.cols; ++j) {
                cv::Point2d pixel;
                float z = img_out.at<float>(i, j);

                cv::Mat res=z*LR.inv()
            }
        }
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
}