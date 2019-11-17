//
// Created by xqj on 2019/11/17.
//

#ifndef ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_LIGHT_BLOB_H_
#define ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_LIGHT_BLOB_H_
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <hwloc/helper.h>
namespace roborts_detection {

class LightBlob {
 public:
  cv::RotatedRect rect;//灯条位置
  double area_ratio;//轮廓面积和其最小外接矩形面积之比
  double length;//灯条长度
  unsigned int blob_color;//灯条颜色
  LightBlob(cv::RotatedRect &rotatedRect, double ratio, uint8_t color);

  LightBlob() = default;

  //旋转矩形的长宽比
  static double lw_rate(const cv::RotatedRect &rect);

  //轮廓面积和其最小外接矩形面积之比
  static double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

  static uint8_t get_blob_color(cv::Mat &src, cv::RotatedRect &blobRect);

  //判断是否是一个灯条
  static bool isVaildLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect);

};

typedef std::vector<LightBlob> LightBlobs;

}
#endif //ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_LIGHT_BLOB_H_
