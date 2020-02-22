//
// Created by wangpei on 2020/1/10.
//
#define FPS_12 2
#define FPS_7 3
#include "camera_convert.h"
#include <ros/ros.h>
#include <cmath>
roborts_camera::camera_convert::camera_convert(
        cv::Mat &depth, cv::Mat &color) {
    float intrinsicL[3][3] = {/*{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
                              {0.,                   9.19093433238763e+02, 3.70129208057065e+02},
                              {0.,                   0.,                   1}*/
            {612.488, 0.,      325.411},
            {0.,      612.729, 246.753},
            {0.,      0.,      1},
    };
    float intrinsicR[3][3] = {/*{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
                              {0.,                   9.19093433238763e+02, 3.70129208057065e+02},
                              {0.,                   0.,                   1},*/
            {612.488, 0.,      325.411},
            {0.,      612.729, 246.753},
            {0.,      0.,      1},
    };
    //realsense and MVS extrinsic mat info
    float extrinsic[3][4] = {/*{9.9930980513982504e-01,  3.5070411883416031e-02,
                                     1.2246614296712125e-02, -2.3640906380346682e-01},
                             {-3.6161562130512234e-02, 9.9384880431654243e-01,
                                     1.0467519086612581e-01, -2.6589124679310618e+00},
                             {-8.5002809180629770e-03, -1.0504580129118116e-01,
                                     9.9443105585827696e-01, 3.4991621301756459e-01},*/
    };
    float rotation[3][3] = {
            /* {9.9930980513982504e-01,  3.5070411883416031e-02,
                     1.2246614296712125e-02},
             {-3.6161562130512234e-02, 9.9384880431654243e-01,
                     1.0467519086612581e-01},
             {-8.5002809180629770e-03, -1.0504580129118116e-01,
                     9.9443105585827696e-01},*/
            {1,  0., 0},
            {0., 1,  0},
            {0., 0., 1},
    };
    float translation[3] = {0, 0, 0
            /* -2.3640906380346682e-01,
             -2.6589124679310618e+00,
             3.4991621301756459e-01,*/
    };
    depth.copyTo(img_depth_src_);
    color.copyTo(img_color_src_);
    intrinsicL_ = cv::Mat(3,3,CV_32F,intrinsicL);
    intrinsicR_ = cv::Mat(3,3,CV_32F,intrinsicR);
    cx_ = 325.411;
    cy_ = 246.753;
    fx_ = 612.488;
    fy_ = 612.729;
    rotation_ = cv::Mat(3,3,CV_32F,rotation);
    translation_ = cv::Mat(3,1,CV_32F,translation);
    XYZ_ = cv::Mat::zeros(3,1,CV_32F);
    uv_  = cv::Mat::zeros(3,1,CV_32F);

//    pixels_count = 0;
    ratio_ = 0;
    color_convert_enable = true;
    height_ = 480;
    width_ = 640;
    pixel_point_colors_.clear();
    //pixel_points_.clear();
    /*std::cout<<intrinsicL_.inv()<<std::endl;
    std::cout<<intrinsicL_<<std::endl;*/
    //hhh
}




roborts_camera::camera_convert::pixelPointColors roborts_camera::camera_convert::get_pixel_points_color_() {
    for (int i = 0; i < img_depth_src_.rows; ++i) {
        auto depth_src_rowptr = img_depth_src_.ptr<ushort>(i);
        for (int j = 0; j < img_depth_src_.cols; ++j) {

            //ROS_ERROR("get ppoint func");
            uv_.at<float>(0, 0) = j;
            uv_.at<float>(1, 0) = i;
            uv_.at<float>(2, 0) = 1;
            float z = depth_src_rowptr[j];

            //ROS_INFO("z::%f",z);
            cv::Mat res(3, 1, CV_32F);

            point3fW_.x = z * ((float) j - cx_) / fx_;

            point3fW_.y = z * ((float) i - cy_) / fy_;

            point3fW_.z = z;

            XYZ_.at<float>(0, 0) = point3fW_.x;

            XYZ_.at<float>(1, 0) = point3fW_.y;

            XYZ_.at<float>(2, 0) = point3fW_.z;

            XYZ_C_ = rotation_ * XYZ_ + translation_;

//            uv_ = intrinsicR_ * res / (z + 0.000000001);
            auto uv_ptr_0 = uv_.ptr<float>(0);


            auto uv_ptr_1 = uv_.ptr<float>(1);

            float u = *uv_ptr_0;

            float v = *uv_ptr_1;
            /*float u = uv_.at<float>(0, 0);
            float v = uv_.at<float>(1, 0);*/
            float r11 = 1, r12 = 0, r13 = 0, r21 = 0, r22 = 1, r23 = 0, t1 = 0, t2 = 0, t3 = 0;
            *uv_ptr_0 = cx_ + fx_ * (r11 * (u -  cx_) / fx_ + r12 * (v - cy_) / fy_ + r13 + t1 / z) * z /
                              z;//注意这里其实时转换后的z被除以才可以
            *uv_ptr_1 = cy_ + fy_ * (r21 * (u - cx_) / fx_ + r22 * (v - cy_) / fy_ + r23 + t2 / z) * z /
                              z;//注意这里其实时转换后的z被除以才可以
            point3fP_.x = *uv_ptr_0; //**********算出u
            point3fP_.y = *uv_ptr_1;//************算出v
            point3fP_.z = z;
            if (point3fP_.x > 0 && point3fP_.y > 0 &&
                point3fP_.y < height_ && point3fP_.x < width_) {
                pixels_count++;
                //pixel_points_.push_back(point3fP_);
                //get the pixel point in the color_img (u,v,z)
                // pixel_point_colors_.pixel_points_in_color.push_back(point3fP_);

                cv::Point3i *pointRGB = new cv::Point3i(img_color_src_.at<cv::Vec3b>(i, j)[0],
                                     img_color_src_.at<cv::Vec3b>(i, j)[1],
                                     img_color_src_.at<cv::Vec3b>(i, j)[2]);
                //get the pixel point with rgb info in the color_img (b,g,r)
                //pixel_point_colors_.pixel_points_rgb.push_back(pointRGB);
                pixelPointColor pointColor;
                pointColor.pixel_points_in_color = point3fP_;
                pointColor.pixel_points_rgb = *pointRGB;
                pixel_point_colors_.push_back(pointColor);
                world_points_.push_back(point3fW_);
                delete pointRGB;

            } else {
                world_points_.push_back(point3fW_);
                continue;
            }
        }
    }
//    ROS_INFO("pixel points cnt: %d\n", pixels_count);
//    ROS_ERROR("depth Ppoint size: %d\n", pixel_points_.size());
    //get the struct we want
    return pixel_point_colors_;
}

cv::Mat roborts_camera::camera_convert::get_depth_dst_new() {


    img_depth_dst_ = cv::Mat::zeros(height_, width_, CV_16UC1);
    ushort *ptr = (ushort *) img_depth_dst_.data;
    for (const auto &pixelPoint : pixel_point_colors_) {
        if (std::isnan(pixelPoint.pixel_points_in_color.z))continue;

        //ROS_INFO("x %f,y %f,z %f", pixelPoint.pixel_points_in_color.x,
         //       pixelPoint.pixel_points_in_color.y,
          //      pixelPoint.pixel_points_in_color.z);

        ptr[(int)(pixelPoint.pixel_points_in_color.y *width_
                + pixelPoint.pixel_points_in_color.x)]
                = (ushort)pixelPoint.pixel_points_in_color.z;

    }
    return img_depth_dst_;
}


cv::Mat roborts_camera::camera_convert::get_color_dst_() {
//i
    img_color_dst_ = cv::Mat::zeros(height_, width_, CV_8UC3);
    auto ptr = (cv::Vec3b *) img_color_dst_.data;
    for (const auto &pointColor : pixel_point_colors_) {
        if (int i = std::isnan(pointColor.pixel_points_in_color.z))continue;
        ptr[(int) (pointColor.pixel_points_in_color.y * width_ + pointColor.pixel_points_in_color.x)][0]
                = (uchar)pointColor.pixel_points_rgb.x;
        ptr[(int) (pointColor.pixel_points_in_color.y * width_ + pointColor.pixel_points_in_color.x)][1]
                = (uchar)pointColor.pixel_points_rgb.y;
        ptr[(int) (pointColor.pixel_points_in_color.y * width_ + pointColor.pixel_points_in_color.x)][2]
                = (uchar)pointColor.pixel_points_rgb.z;

    }

    return img_color_dst_;
}

