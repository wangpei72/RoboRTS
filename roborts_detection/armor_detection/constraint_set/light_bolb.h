//
// Created by xqj on 2019/11/17.
//

#ifndef ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_LIGHT_BLOB_H_
#define ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_LIGHT_BLOB_H_
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
namespace roborts_detection {

class LightBolb {
 public:
  cv::RotatedRect rect;//灯条位置
  double area_ratio;//轮廓面积和其最小外接矩形面积之比
  double length;//灯条长度
  unsigned int bolb_color;//灯条颜色
  LightBolb(cv::RotatedRect &rotatedRect, double ratio, uint8_t color);

  LightBolb() = default;

  //旋转矩形的长宽比
  static double lw_rate(const cv::RotatedRect &rect);

  //轮廓面积和其最小外接矩形面积之比
  static double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

  //判断是否是一个灯条
  static bool isValidLightBolb(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

  static double light_area_threshold;
  static double light_small_ratio;
  static double light_big_ratio;
  static double light_small_lw_rate;
  static double light_big_lw_rate;

};

typedef std::vector<LightBolb> LightBolbs;

}
#endif //ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_LIGHT_BLOB_H_
