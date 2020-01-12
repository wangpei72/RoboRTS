//
// Created by wangpei on 2020/1/10.
//
/**  data: 612.488
    data: 0.0
    data: 325.411
    data: 0.0
    data: 612.729
    data: 246.753
    data: 0.0
    data: 0.0
    data: 1.0*/

#include "camera_convert.h"

roborts_camera::camera_convert::camera_convert(
        cv::Mat &depth) {
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
    intrinsicL_ = cv::Mat(3,3,CV_32F,intrinsicL);
    intrinsicR_ = cv::Mat(3,3,CV_32F,intrinsicR);
    cx_ = 325.411;
    cy_ = 246.753;
    fx_ = 612.488;
    fy_ = 612.729;
    /*intrinsicL_ = cv::Mat_<float>(3, 3)<< 1,0,0,0,1,0,0,0,1;*/
//    extrinsic_ = cv::Mat(3,4,CV_32F,extrinsic);
    rotation_ = cv::Mat(3,3,CV_32F,rotation);
    translation_ = cv::Mat(3,1,CV_32F,translation);
    XYZ_ = cv::Mat::zeros(3,1,CV_32F);
    uv_  = cv::Mat::zeros(3,1,CV_32F);
    world_points_.resize(600);
    pixel_points_.resize(600);
    pixels_count = 0;
    ratio_ = 0;
    color_convert_enable = true;
    height_ = 480;
    width_ = 640;/*std::cout<<intrinsicL_.inv()<<std::endl;
    std::cout<<intrinsicL_<<std::endl;*/
}

/*roborts_camera::camera_convert::camera_convert(
        cv::Mat &depth, cv::Mat &color) {
    float intrinsicL[3][3] = {{9.18732064958717, 0.,                   6.481170182761775},
                              {0.,                   9.19093433238763, 3.70129208057065},
                              {0.,                   0.,                   1},
    };
    float intrinsicR[3][3] = {{9.18732064958717, 0.,                   6.481170182761775},
                              {0.,                   9.19093433238763, 3.70129208057065},
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
    //depth.copyTo(img_depth_src_);
    //color.copyTo(img_color_src_);
    intrinsicL_ = cv::Mat_<float>(3, 3)<< 1,0,0,0,1,0,0,0,1;
    intrinsicR_ = cv::Mat(3, 3, CV_32F, intrinsicR);
//    extrinsic_ = cv::Mat(3, 4, CV_32F, extrinsic);
    rotation_ = cv::Mat(3, 3, CV_32F, rotation);
    translation_ = cv::Mat(3, 1, CV_32F, translation);
    XYZ_ = cv::Mat::zeros(3, 1, CV_32F);
    uv_ = cv::Mat::zeros(3, 1, CV_32F);
    world_points_.resize(600);
    pixel_points_.resize(600);
    pixels_count = 0;
    color_convert_enable = true;
    pixel_point_colors_.resize(600);
    height_ = 2048;
    width_ = 3072;
    std::cout<<intrinsicL_.inv()<<std::endl;
    std::cout<<intrinsicL_<<std::endl;
}*/

std::vector<cv::Point3f> roborts_camera::camera_convert::get_pixel_points_() {
    for (int i = 0; i < img_depth_src_.rows; ++i) {
        for (int j = 0; j < img_depth_src_.cols; ++j) {
           uv_.at<float>(0,0)=j;
           uv_.at<float>(1,0)=i;
           uv_.at<float>(2,0)=1;
            float z = img_depth_src_.at<ushort>(i, j);
//            printf("z: %f\n", z);
            //test *************************************************以下部分是修改开始******************
            cv::Mat res = cv::Mat(3, 1, CV_32F); /*= (z + 0.000001) * intrinsicL_.inv() * uv_*/
            /*std::cout<<intrinsicL_.inv()<<std::endl;
            std::cout<<intrinsicL_<<std::endl;*/
//           res = rotation_*res + translation_;

            /*XYZ_ = res;*/
            /*point3fW_.x = res.at<float>(0,0);
            point3fW_.y = res.at<float>(1,0);
             point3fW_.z = XYZ_.at<float>(2, 0);*/
            /*  if (point3fW_.z == z) {
                  printf("z eqauls to z");
              } else printf("z not right");*/
           //uv_ get transformed
//           uv_ = intrinsicR_ * res / (z + 0.000001);
            point3fW_.x = z * (uv_.at<float>(0, 0) - cx_) / fx_;//算出x
            point3fW_.y = z * (uv_.at<float>(1, 0) - cy_) / fy_;//算出y
            point3fW_.z = z;//z
            XYZ_.at<float>(0, 0) = point3fW_.x;
            XYZ_.at<float>(1, 0) = point3fW_.y;
            XYZ_.at<float>(2, 0) = point3fW_.z;
            XYZ_C_ = rotation_ * XYZ_ + translation_;
            //不过暂时先不用rt后的xyz坐标
            float u = uv_.at<float>(0, 0);
            float v = uv_.at<float>(1, 0);
            float r11 = 1, r12 = 0, r13 = 0, r21 = 0, r22 = 1, r23 = 0, t1 = 0, t2 = 0, t3 = 0;
            uv_.at<float>(0, 0) = cx_ + fx_ * (r11 * (u - cx_) / fx_ + r12 * (v - cy_) / fy_ + r13 + t1 / z) * z /
                                        z;//注意这里其实时转换后的z被除以才可以
            uv_.at<float>(1, 0) = cy_ + fy_ * (r21 * (u - cx_) / fx_ + r22 * (v - cy_) / fy_ + r23 + t2 / z) * z /
                                        z;//注意这里其实时转换后的z被除以才可以
            point3fP_.x = uv_.at<float>(0, 0);//算出U
            point3fP_.y = uv_.at<float>(1, 0);//算出V
            point3fP_.z = z;//带上深度信息z
//           printf("u: %f v: %f  z: %f  \n ",point3fP_.x,point3fP_.y,point3fP_.z);

           if(point3fP_.x >0 && point3fP_.y >0 &&
              point3fP_.y < 480 && point3fP_.x < 640) {
               pixels_count++;

               pixel_points_.push_back(point3fP_);
           }
           world_points_.push_back(point3fW_);
        }
    }
//    printf("%d\n",pixels_count);
    return pixel_points_;
}

roborts_camera::camera_convert::pixelPointColors roborts_camera::camera_convert::get_pixel_points_color_(
        cv::Mat &depth, cv::Mat &color) {
    for (int i = 0; i < depth.rows; ++i) {
        for (int j = 0; j < depth.cols; ++j) {
            uv_.at<float>(0, 0) = j;
            uv_.at<float>(1, 0) = i;
            uv_.at<float>(2, 0) = 1;
            float z = depth.at<ushort>(i, j);
            cv::Mat res = z * intrinsicL_.inv() * uv_;
            res = rotation_ * res + translation_;
            XYZ_ = res;
            point3fW_.x = res.at<float>(0, 0);
            point3fW_.y = res.at<float>(1, 0);
            point3fW_.z = XYZ_.at<float>(2, 0);
            /* if (point3fW_.z == z) {
                 printf("z eqauls to z");

             } else printf("z not right");*/
            //uv_ get transformed
            uv_ = intrinsicR_ * res / (z + 0.000000001);
            point3fP_.x = uv_.at<float>(0, 0);
            point3fP_.y = uv_.at<float>(1, 0);
            point3fP_.z = z;
            if (point3fP_.x > 0 && point3fP_.y > 0 &&
                point3fP_.y < 2048 && point3fP_.x < 3072) {

                pixel_points_.push_back(point3fP_);
                //get the pixel point in the color_img (u,v,z)
                // pixel_point_colors_.pixel_points_in_color.push_back(point3fP_);
                world_points_.push_back(point3fW_);
                cv::Point3i pointRGB(color.at<cv::Vec3b>(i, j)[0],
                                     color.at<cv::Vec3b>(i, j)[1],
                                     color.at<cv::Vec3b>(i, j)[2]);
                //get the pixel point with rgb info in the color_img (b,g,r)
                //pixel_point_colors_.pixel_points_rgb.push_back(pointRGB);
                pixelPointColor pointColor;
                pointColor.pixel_points_in_color = point3fP_;
                pointColor.pixel_points_rgb = pointRGB;
                pixel_point_colors_.push_back(pointColor);
                pixels_count++;
            } else continue;
        }
    }
    printf("%d\n", pixels_count);
    //get the struct we want
    return pixel_point_colors_;
}

cv::Mat roborts_camera::camera_convert::get_depth_dst_() {

    ratio_ = (width_ * height_) / (pixel_points_.size());
    ratio_ = pow(ratio_, 0.5);
    if (ratio_ % 2 == 0) {
        ratio_ += 1;
    }
    img_depth_dst_.create(height_, width_, CV_32F);

    for (const auto &pixelPoint : pixel_points_) {
        img_depth_dst_.at<float>(pixelPoint.y, pixelPoint.x) = pixelPoint.z;
    }
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::dilate(img_depth_dst_, img_depth_dst_, element);
    return img_depth_dst_;
}

cv::Mat roborts_camera::camera_convert::get_color_dst_() {

    ratio_ = (width_ * height_) / (pixel_points_.size());
    ratio_ = pow(ratio_, 0.5);
    if (ratio_ % 2 == 0) {
        ratio_ += 1;
    }
    img_color_dst_.create(height_, width_, CV_8UC3);
    for (const auto &pointColor : pixel_point_colors_) {
        img_color_dst_.at<cv::Vec3b>(pointColor.pixel_points_in_color.y, pointColor.pixel_points_in_color.x)[0]
                = pointColor.pixel_points_rgb.x;
        img_color_dst_.at<cv::Vec3b>(pointColor.pixel_points_in_color.y, pointColor.pixel_points_in_color.x)[1]
                = pointColor.pixel_points_rgb.y;
        img_color_dst_.at<cv::Vec3b>(pointColor.pixel_points_in_color.y, pointColor.pixel_points_in_color.x)[2]
                = pointColor.pixel_points_rgb.z;
    }
/*
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ratio_, ratio_));
    cv::dilate(img_color_dst_, img_color_dst_, element);
    *///cv::GaussianBlur(img_depth_dst_, img_depth_dst_, cv::Size(ratio_, ratio_), 0);

    return img_color_dst_;

}