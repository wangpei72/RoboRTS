//
// Created by wangpei on 2020/1/10.
//

#ifndef SRC_CAMERA_CONVERT_H
#define SRC_CAMERA_CONVERT_H


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
namespace roborts_camera {

    class camera_convert {
    public:
        int pixels_count{};
        int width_;
        int height_;
        cv::Mat img_depth_src_;
        cv::Mat img_color_src_;
        cv::Mat img_depth_dst_;
        cv::Mat img_color_dst_;
        int ratio_{};//工业相机密集像素点个数/深度相机投影点个数
        cv::Mat intrinsicL_;
        cv::Mat intrinsicR_;
        float cx_, cy_;
        float fx_, fy_;
//        cv::Mat extrinsic_;
        cv::Mat rotation_;
        cv::Mat translation_;
        bool color_convert_enable{};
        cv::Mat XYZ_C_;//convert result world point
        cv::Mat XYZ_;//origin world point
        cv::Mat uv_;//transformed pixel result
        //x y z stands for real world coordinate
        std::vector<cv::Point3f> world_points_;
        cv::Point3f point3fW_;
        //u v z stands for img on MVS: (u,v) depth =z
        cv::Point3f point3fP_;
        typedef struct {
            cv::Point3f pixel_points_in_color;
            cv::Point3i pixel_points_rgb;
        } pixelPointColor;
        typedef std::vector<pixelPointColor> pixelPointColors;

        pixelPointColors pixel_point_colors_;
        std::vector<cv::Point3f> pixel_points_;

//        这个参数是转换后深度图和彩色图通用的带深度信息的像素点
        camera_convert()= default;
        virtual ~camera_convert()= default;

//        camera_convert(cv::Mat &depth);

        camera_convert(cv::Mat &depth, cv::Mat &color);

        std::vector<cv::Point3f> get_pixel_points_();

        pixelPointColors get_pixel_points_color_();
        cv::Mat get_depth_dst_();

        cv::Mat get_color_dst_();
    };

}
#endif //SRC_CAMERA_CONVERT_H
