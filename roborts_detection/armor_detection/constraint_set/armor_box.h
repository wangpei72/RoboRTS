//
// Created by xqj on 2019/11/22.
//

#ifndef ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_ARMOR_BOX_H_
#define ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_ARMOR_BOX_H_
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "light_bolb.h"

namespace roborts_detection {
class ArmorBox {
 public:

  typedef enum {
    FRONT, SIDE, BACK
  } BoxOrientation;

  cv::Rect2d rect;
  cv::Point center;
  roborts_detection::LightBolbs light_bolbs;
  uint8_t box_color;
  int id;

  ArmorBox(cv::Rect2d rect_2_d, LightBolbs light_bolbs1, uint8_t box_color_init,
           int id_init = 1);


  //以下用于灯管匹配的函数

  static bool lengthRatioJudge(const LightBolb &lightBolb_i, const LightBolb &lightBolb_j);

  static bool lengthJudge(const LightBolb &lightBolb_i, const LightBolb &lightBolb_j);

  static bool angleJudge(const LightBolb &lightBolb_i, const LightBolb &lightBolb_j);

  static bool isCoupleLight(const LightBolb &light_bolb_i, const LightBolb &light_bolb_j, uint8_t enemy_color);

  bool isMatchArmorBox();
};

typedef std::vector<ArmorBox> ArmorBoxs;
}
#endif //ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_ARMOR_BOX_H_
