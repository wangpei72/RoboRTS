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

double lw_rate(const cv::RotatedRect &rect) {
  return rect.size.height > rect.size.width ?
         rect.size.height / rect.size.width :
         rect.size.width / rect.size.height;
}
double areaRatio(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
  return contourArea(contour) / rect.size.area();
}

// 判断灯条颜色(此函数可以有性能优化).
// TODO
uint8_t get_blob_color(cv::Mat &src, cv::RotatedRect &blobPos) {
  auto region = blobPos.boundingRect();
  region.x -= fmax(3, region.width * 0.1);
  region.y -= fmax(3, region.height * 0.05);
  region.width += 2 * fmax(3, region.width * 0.1);
  region.height += 2 * fmax(3, region.height * 0.05);
  region &= cv::Rect(0, 0, src.cols, src.rows);
  cv::Mat roi = src(region);
  int red_cnt = 0, blue_cnt = 0, white_cnt = 0;
  for (int row = 0; row < roi.rows; row++) {
    for (int col = 0; col < roi.cols; col++) {
      red_cnt += roi.at<cv::Vec3b>(row, col)[2];
      blue_cnt += roi.at<cv::Vec3b>(row, col)[0];
      if (roi.at<cv::Vec3b>(row, col)[0] > 200
          && roi.at<cv::Vec3b>(row, col)[1] > 200
          && roi.at<cv::Vec3b>(row, col)[2] > 200) {
        white_cnt++;
      }
    }
  }
  if (white_cnt > roi.rows * roi.cols * 0.8) {
    //白色
    return -1;
  }
  if (red_cnt > blue_cnt) {
    //红色
    return 1;
  } else {
    return 0;
  }
}

bool isVaildLightBlob(const std::vector<cv::Point> &contour, const cv::RotatedRect &rect) {
  return (1.2 < lw_rate(rect) && lw_rate(rect) < 10) &&
      ((rect.size.area() < 50 && areaRatio(contour, rect) > 0.4) ||
          rect.size.area() >= 50 && areaRatio(contour, rect) > 0.6);
}

