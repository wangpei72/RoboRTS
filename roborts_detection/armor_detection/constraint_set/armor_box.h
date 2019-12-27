//
// Created by xqj on 2019/11/22.
//

#ifndef ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_ARMOR_BOX_H_
#define ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_ARMOR_BOX_H_
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "light_blob.h"

namespace roborts_detection {
class ArmorBox {
 public:

  typedef enum {
    FRONT, SIDE, BACK
  } BoxOrientation;

  cv::Rect2d rect;
  cv::Point center;
  roborts_detection::LightBlobs light_blobs;
  uint8_t box_color;
  int id;

  ArmorBox(cv::Rect2d rect_2_d, LightBlobs light_blobs1, uint8_t box_color_init,
           int id_init = 2);

  //构造一个存在的装甲板
  ArmorBox(int id_inti = -1);

  //以下用于灯管匹配的函数

  static bool lengthRatioJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j);

  static bool lenghtJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j);

  static bool angelJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j);

  static bool isCoupleLight(const LightBlob &light_blob_i, const LightBlob &light_blob_j, uint8_t enemy_color);

  bool isMatchArmorBox();
};

typedef std::vector<ArmorBox> ArmorBoxs;
}
#endif //ROBORTS_WS_SRC_ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_ARMOR_BOX_H_
