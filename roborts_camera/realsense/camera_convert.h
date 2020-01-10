//
// Created by wangpei on 2020/1/10.
//

#ifndef SRC_CAMERA_CONVERT_H
#define SRC_CAMERA_CONVERT_H


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>cam
#include <cv_bridge/cv_bridge.h>
#include <vector>
namespace roborts_camera {

    class camera_convert {
    public:
        int pixels_count;
        cv::Mat img_depth_src_;
        cv::Mat img_depth_dst_;
        int ratio_;//工业相机密集像素点个数/深度相机投影点个数
        cv::Mat intrinsicL_;
        cv::Mat intrinsicR_;
        cv::Mat extrinsic_;
        cv::Mat rotation_;
        cv::Mat translation_;

        cv::Mat XYZ_;//world points result
        cv::Mat uv_;//transformed pixel result
        //x y z stands for real world coordinate
        std::vector<cv::Point3f> world_points_;
        cv::Point3f point3fW_;
        //u v z stands for img on MVS: (u,v) depth =z
        cv::Point3f point3fP_;
        std::vector<cv::Point3f> pixel_points_;
        camera_convert()= default;
        virtual ~camera_convert()= default;
        camera_convert(cv::Mat &img);
        std::vector<cv::Point3f> get_pixel_points_(cv::Mat &img);
        cv::Mat get_depth_dst_(cv::Mat &img);
    };

}
#endif //SRC_CAMERA_CONVERT_H
