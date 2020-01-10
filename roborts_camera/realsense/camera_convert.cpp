//
// Created by wangpei on 2020/1/10.
//

#include "camera_convert.h"
roborts_camera::camera_convert::camera_convert(
        cv::Mat &img){
    float intrinsicL[3][3] = {{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
                              {0.,                   9.19093433238763e+02, 3.70129208057065e+02},
                              {0.,                   0.,                   1},
    };
    float intrinsicR[3][3] = {{9.18732064958717e+02, 0.,                   6.481170182761775e+02},
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
    img.copyTo(img_depth_src_);
    intrinsicL_ = cv::Mat(3,3,CV_32F,intrinsicL);
    intrinsicR_ = cv::Mat(3,3,CV_32F,intrinsicR);
    extrinsic_ = cv::Mat(3,4,CV_32F,extrinsic);
    rotation_ = cv::Mat(3,3,CV_32F,rotation);
    translation_ = cv::Mat(3,1,CV_32F,translation);
    XYZ_ = cv::Mat::zeros(3,1,CV_32F);
    uv_  = cv::Mat::zeros(3,1,CV_32F);
    world_points_.clear();
    pixel_points_.clear();
    pixels_count = 0;

}

std::vector<cv::Point3f> roborts_camera::camera_convert::get_pixel_points_(cv::Mat &img) {
    for (int i = 0; i < img.rows; ++i) {
        for (int j = 0; j < img.cols; ++j) {
           uv_.at<float>(0,0)=j;
           uv_.at<float>(1,0)=i;
           uv_.at<float>(2,0)=1;
           float z = img.at<float>(i,j);
           cv::Mat res = z*intrinsicL_.inv()*uv_;
           res = rotation_*res + translation_;
           point3fW_.x = res.at<float>(0,0);
           point3fW_.y = res.at<float>(1,0);
           point3fW_.z = z ;
           //uv_ get transformed
           uv_ = intrinsicR_*res /z ;
           point3fP_.x = uv_.at<float>(0,0);
           point3fP_.y = uv_.at<float>(1,0);
           point3fP_.z = z ;
           if(point3fP_.x >0 && point3fP_.y >0&&
           point3fP_.y<2048&&point3fP_.x<3072){
               pixels_count++;

               pixel_points_.push_back(point3fP_);
           }
           world_points_.push_back(point3fW_);
        }
    }
    printf("%d\n",pixels_count);
    return pixel_points_;
}
