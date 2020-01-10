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
        cv::Mat img_depth_src_;
        cv::Mat img_depth_dst_;
        double ratio_;//工业相机密集像素点个数/深度相机投影点个数
        static cv::Mat intrinsicL_;
        static cv::Mat intrinsicR_;
        cv::Mat extrinsic_;
        cv::Mat rotation_;
        cv::Mat translation_;
        cv::Mat XYZ_;
        cv::Mat uv_;
        std::vector<cv::Point3f> world_points_;
        std::vector<cv::Point2f> pixel_points_;

    };

}
#endif //SRC_CAMERA_CONVERT_H
