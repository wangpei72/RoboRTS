//
// Created by wangpei on 2020/3/6.
//

#include "tail_blob.h"

roborts_detection::TailBlob::TailBlob(cv::RotatedRect &rotatedRect, double ratio, uint8_t color) {
    rect_ = rotatedRect;
    area_ratio_ = ratio;
    blob_color_ = color;
    length_ = std::max(rotatedRect.size.height, rotatedRect.size.width);
}

double roborts_detection::TailBlob::lw_rate(const cv::RotatedRect &rect) {
    return rect.size.height / rect.size.width;
}

double roborts_detection::TailBlob::area_ratio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
    return cv::contourArea(contour) / rect.size.area();
}

bool roborts_detection::TailBlob::isValidTailBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
    return (0 < lw_rate(rect) && lw_rate(rect) < 0.25) &&
           ((rect.size.area() < 90 && area_ratio(contour, rect) > 0.4) ||
            (rect.size.area() >= 90 && area_ratio(contour, rect) > 0.6));
}