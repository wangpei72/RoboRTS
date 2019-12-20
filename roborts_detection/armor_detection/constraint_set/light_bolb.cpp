//
// Created by xqj on 2019/11/17.
//

#include "light_bolb.h"
roborts_detection::LightBolb::LightBolb(cv::RotatedRect &rotatedRect,
        double ratio,
        uint8_t color) {
  rect = rotatedRect;
  area_ratio = ratio;
  length = std::max(rotatedRect.size.height, rotatedRect.size.width);
  bolb_color = color;
}

double roborts_detection::LightBolb::lw_rate(const cv::RotatedRect &rect) {
    /*长宽比*/
  return rect.size.height > rect.size.width ?
         rect.size.height / rect.size.width :
         rect.size.width / rect.size.height;
}

double roborts_detection::LightBolb::areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
  return contourArea(contour) / rect.size.area();
}

bool roborts_detection::LightBolb::isValidLightBolb(const std::vector<cv::Point> &contour,
                                                    const cv::RotatedRect &rect) {
  return (1 < lw_rate(rect) && lw_rate(rect) < 10) &&
      ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
          rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6);
}
