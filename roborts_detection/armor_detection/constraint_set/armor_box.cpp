//
// Created by xqj on 2019/11/22.
//
#include "armor_box.h"

bool roborts_detection::ArmorBox::lengthRatioJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j) {
  bool judge = (lightBlob_i.length / lightBlob_i.length < 1.5 &&
      lightBlob_i.length / lightBlob_j.length > 1 / 1.5);
  if (!judge) {
    printf("LengthRadioNot  ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::lenghtJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j) {
  double side_lenght;
  cv::Point2f center = lightBlob_i.rect.center - lightBlob_j.rect.center;
  side_lenght = sqrt(center.ddot(center));
  bool judge = (side_lenght / lightBlob_i.length < 10 && side_lenght / lightBlob_i.length
      && side_lenght / lightBlob_j.length < 10 && side_lenght / lightBlob_j.length);
  if (!judge) {
    printf("LenghtNot ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::angelJudge(const LightBlob &lightBlob_i, const LightBlob &lightBlob_j) {
  float angle_i = lightBlob_i.rect.size.width > lightBlob_i.rect.size.height ? lightBlob_i.rect.angle :
                  lightBlob_i.rect.angle - 90;
  float angle_j = lightBlob_j.rect.size.width > lightBlob_j.rect.size.height ? lightBlob_j.rect.angle :
                  lightBlob_j.rect.angle - 90;
  bool judge = abs(angle_i - angle_j) < 20;
  if (!judge) {
    printf("angleNot ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::isCoupleLight(const LightBlob &light_blob_i,
                                                const LightBlob &light_blob_j,
                                                uint8_t enemy_color) {
  return light_blob_i.blob_color == enemy_color &&
      light_blob_j.blob_color == enemy_color &&
      lengthRatioJudge(light_blob_i, light_blob_j) &&
      lenghtJudge(light_blob_i, light_blob_j) &&
      angelJudge(light_blob_i, light_blob_j);
}

roborts_detection::ArmorBox::ArmorBox(cv::Rect2d rect,
                                      roborts_detection::LightBlobs light_blobs,
                                      uint8_t box_color,
                                      int id) {
  rect = rect;
  light_blobs = light_blobs;
  box_color = box_color;
  id = id;
}

