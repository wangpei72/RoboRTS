//
// Created by wangpei on 2020/3/6.
//

#ifndef SRC_TAIL_BLOB_H
#define SRC_TAIL_BLOB_H


#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

namespace roborts_detection {

    class TailBlob {
    public:
        cv::RotatedRect rect_;
        double area_ratio_;
        double length_;
        unsigned int blob_color_;

        TailBlob(cv::RotatedRect &rotatedRect, double ratio, uint8_t color);

        TailBlob() = default;

        static double lw_rate(const cv::RotatedRect &rect);

        static double area_ratio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

        static bool isValidTailBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

        static double light_area_threshold;
        static double light_small_ratio;
        static double light_big_ratio;
        static double light_small_lw_rate;
        static double light_big_lw_rate;
    };

    typedef std::vector<TailBlob> TailBlobs;
}


#endif //SRC_TAIL_BLOB_H
