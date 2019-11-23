//
// Created by xqj on 2019/11/17.
//

#include "light_blob.h"
roborts_detection::LightBlob::LightBlob(cv::RotatedRect &rotatedRect, double ratio, uint8_t color) {
  rect = rotatedRect;
  area_ratio = ratio;
  length = std::max(rotatedRect.size.height, rotatedRect.size.width);
  blob_color = color;
}

double roborts_detection::LightBlob::lw_rate(const cv::RotatedRect &rect) {
  return rect.size.height > rect.size.width ?
         rect.size.height / rect.size.width :
         rect.size.width / rect.size.height;
}

double roborts_detection::LightBlob::areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
  return contourArea(contour) / rect.size.area();
}

bool roborts_detection::LightBlob::isVaildLightBlob(const std::vector<cv::Point> &contour,
                                                    const cv::RotatedRect &rect) {
  return (3 < lw_rate(rect) && lw_rate(rect) < 6) &&
      ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
          rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6);
}
