//
// Created by xqj on 2019/11/22.
//
#include "armor_box.h"

bool roborts_detection::ArmorBox::lengthRatioJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j) {
  bool judge = (lightBlob_i.length / lightBlob_j.length < 1.5 &&
      lightBlob_i.length / lightBlob_j.length > 1 / 1.5);
  //修复bug 下标错误
  if (!judge) {
//    printf("LengthRadioNot  ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::lengthJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j) {
  double side_length;
  cv::Point2f center = lightBlob_i.rect.center - lightBlob_j.rect.center;
  side_length = sqrt(center.ddot(center));
  bool judge = (side_length / lightBlob_i.length < 10 && side_length / lightBlob_i.length
      && side_length / lightBlob_j.length < 10 && side_length / lightBlob_j.length);
  if (!judge) {
//    printf("LengthNot ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::angleJudge(const LightBlob &lightBlob_i,
                                             const LightBlob &lightBlob_j) {
  float angle_i = lightBlob_i.rect.size.width > lightBlob_i.rect.size.height ? lightBlob_i.rect.angle :
                  lightBlob_i.rect.angle-90 ;
  printf("i angleorigin/angle :%f / %f",lightBlob_i.rect.angle,angle_i);
  float angle_j = lightBlob_j.rect.size.width > lightBlob_j.rect.size.height ? lightBlob_j.rect.angle :
                  lightBlob_j.rect.angle-90;
  bool judge = abs(angle_i - angle_j) < 10;
  printf("j angleorigin/angle :%f / %f",lightBlob_j.rect.angle,angle_j);
  if (!judge) {
//    printf("angleNot ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::isCoupleLight(const LightBlob &light_blob_i,
                                                const LightBlob &light_blob_j,
                                                uint8_t enemy_color) {
//  printf("enemr color is %d enemy color is %d and target is %d\n",
//         light_blob_i.blob_color,
//         light_blob_j.blob_color,
//         enemy_color);
  return light_blob_i.blob_color == enemy_color &&
      light_blob_j.blob_color == enemy_color &&
      lengthRatioJudge(light_blob_i, light_blob_j) &&
      lengthJudge(light_blob_i, light_blob_j) &&
      angleJudge(light_blob_i, light_blob_j);
}


bool roborts_detection::ArmorBox::isMatchArmorBox() {
  //初步判断 长宽比例
  if (this->rect.height / this->rect.width > 1.5
      || this->rect.height / this->rect.width < 1 / 1.5) {
    return false;
  } else
    return true;
}


roborts_detection::ArmorBox::ArmorBox(cv::Rect2d rect_2_d,
                                      roborts_detection::LightBlobs light_blobs1,
                                      uint8_t box_color_init,
                                      int id_init) {
  rect = rect_2_d;
  light_blobs = light_blobs1;
  box_color = box_color_init;
  id = id_init;
  center.x = rect_2_d.x + cvRound(rect_2_d.width / 2.0);
  center.y = rect_2_d.y + cvRound(rect_2_d.height / 2.0);
  //修改bug center.x重复
}

roborts_detection::ArmorBox::ArmorBox(int id_init) {
  id = id_init;
}
