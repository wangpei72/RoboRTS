//
// Created by xqj on 2019/11/22.
//
#include "armor_box.h"

bool roborts_detection::ArmorBox::lengthRatioJudge(const LightBolb &lightBolb_i, const LightBolb &lightBolb_j) {
  bool judge = (lightBolb_i.length / lightBolb_j.length < 1.5 &&
      lightBolb_i.length / lightBolb_j.length > 1 / 1.5);
  //修复bug 下标错误
  if (!judge) {
//    printf("LengthRadioNot  ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::lengthJudge(const LightBolb &lightBolb_i, const LightBolb &lightBolb_j) {
  double side_length;
  cv::Point2f center = lightBolb_i.rect.center - lightBolb_j.rect.center;
    side_length = sqrt(center.ddot(center));
  bool judge = (side_length / lightBolb_i.length < 10 && side_length / lightBolb_i.length
                && side_length / lightBolb_j.length < 10 && side_length / lightBolb_j.length);
  if (!judge) {
//    printf("LengthNot ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::angleJudge(const LightBolb &lightBolb_i,
        const LightBolb &lightBolb_j) {
  float angle_i = lightBolb_i.rect.size.width > lightBolb_i.rect.size.height ? lightBolb_i.rect.angle :
                  lightBolb_i.rect.angle - 90;
  float angle_j = lightBolb_j.rect.size.width > lightBolb_j.rect.size.height ? lightBolb_j.rect.angle :
                  lightBolb_j.rect.angle - 90;
  bool judge = abs(angle_i - angle_j) < 20;
  if (!judge) {
//    printf("angleNot ");
  }
  return judge;
}

bool roborts_detection::ArmorBox::isCoupleLight(const LightBolb &light_bolb_i,
                                                const LightBolb &light_bolb_j,
                                                uint8_t enemy_color) {
//  printf("enemr color is %d enemy color is %d and target is %d\n",
//         light_bolb_i.bolb_color,
//         light_bolb_j.bolb_color,
//         enemy_color);
  return light_bolb_i.bolb_color == enemy_color &&
      light_bolb_j.bolb_color == enemy_color &&
         lengthRatioJudge(light_bolb_i, light_bolb_j) &&
         lengthJudge(light_bolb_i, light_bolb_j) &&
          angleJudge(light_bolb_i, light_bolb_j);
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
                                      roborts_detection::LightBolbs light_bolbs1,
                                      uint8_t box_color_init,
                                      int id_init) {
  rect = rect_2_d;
  light_bolbs = light_bolbs1;
  box_color = box_color_init;
  id = id_init;
  center.x = rect_2_d.x + cvRound(rect_2_d.width / 2.0);
  center.y = rect_2_d.y + cvRound(rect_2_d.height / 2.0);
  //修改bug center.x重复
}

roborts_detection::ArmorBox::ArmorBox(int id_init) {
  id = id_init;
}
