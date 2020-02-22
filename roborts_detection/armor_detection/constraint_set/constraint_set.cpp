/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <algorithm>

#include "constraint_set.h"

#include "timer/timer.h"
#include "io/io.h"

namespace roborts_detection {

ConstraintSet::ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox) :
    ArmorDetectionBase(cv_toolbox) {
  filter_x_count_ = 0;
  filter_y_count_ = 0;
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  read_index_ = -1;
  detection_time_ = 0;
  thread_running_ = false;

  nh = ros::NodeHandle();
  LoadParam();
  error_info_ = ErrorInfo(roborts_common::OK);

  state = SEARCHING_STATE;

}

void ConstraintSet::LoadParam() {
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name = ros::package::getPath("roborts_detection") + \
      "/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  enable_debug_ = constraint_set_config_.enable_debug();
  enemy_color_ = constraint_set_config_.enemy_color();
  using_hsv_ = constraint_set_config_.using_hsv();

  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);

  //algorithm threshold parameters
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().light_max_angle();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
  armor_max_mean_ = constraint_set_config_.threshold().armor_max_mean();

  color_thread_ = constraint_set_config_.threshold().color_thread();
  blue_thread_ = constraint_set_config_.threshold().blue_thread();
  red_thread_ = constraint_set_config_.threshold().red_thread();

  int get_intrinsic_state = -1;
  int get_distortion_state = -1;

  //test
//  while ((get_intrinsic_state < 0) || (get_distortion_state < 0)) {
//    ROS_WARN("Wait for camera driver launch %d", get_intrinsic_state);
//    usleep(50000);
//    ros::spinOnce();
//    get_intrinsic_state = cv_toolbox_->GetCameraMatrix(intrinsic_matrix_);
//    get_distortion_state = cv_toolbox_->GetCameraDistortion(distortion_coeffs_);
//  }
  getCameraInfo("Debug");
}

//暂时保留原框架的DetectArmor,所以命名NewDetectArmor,后期没有借鉴价值可以换名
ErrorInfo ConstraintSet::NewDetectArmor(bool &detected, cv::Point3f &target_3d) {
  //在此处同时更新工业相机和realSense深度图
  src_industry_clone = src_industry_img_.clone();
  src_realSense_depth_clone = src_realSense_depth_img_.clone();
  src_realSense_RGB_clone = src_realSense_RGB_img_.clone();
  switch (state) {
    case SEARCHING_STATE:
      SearchArmor(src_industry_clone,
                  src_realSense_RGB_clone,
                  src_realSense_depth_clone,
                  src_industry_clone,
                  detected,
                  target_3d);
      if (detected) {
        tracker = cv::TrackerKCF::create();
        //在较为清晰的工业相机进行init
        tracker->init(src_industry_clone, possibleBox.rect);
        state = TRACKING_STATE;
        tracking_cnt = 0;
      }
      break;
      //进入跟踪状态，执行跟踪函数
    case TRACKING_STATE:
      trackingTarget(src_industry_clone,
                     src_realSense_RGB_clone,
                     src_realSense_depth_clone,
                     detected,
                     target_3d);
      //未找到或者跟踪时间过长
      if (!detected || tracking_cnt++ == 100) {
        state = SEARCHING_STATE;
        tracking_cnt = 0;
      } else {
        tracker->init(src_industry_clone, possibleBox.rect);
      }
      break;
    default:break;
  }
  //原框架中有ErrorInfo类，后面再看实际情况是否需要这部分
  ErrorInfo error_info = ErrorInfo();
  return
      error_info;
}

//获取相机图片信息
void ConstraintSet::getCameraInfo(std::string info) {
  //如果采用debug模式，则使用realSense相机代替工业相机获取彩图信息
  if (info == "Debug") {
    industrySubscriber = nh.subscribe<sensor_msgs::ImageConstPtr>("/realsense_camera/image_raw",
                                                                  1,
                                                                  &ConstraintSet::getIndustryMat, this);
  } else {
    industrySubscriber = nh.subscribe<sensor_msgs::ImageConstPtr>("/image/bgr_image",
                                                                  1,
                                                                  &ConstraintSet::getIndustryMat, this);
  }
  realSenseDepthSubscriber = nh.subscribe<sensor_msgs::ImageConstPtr>("/realsense_camera/depth_raw",
                                                                      1,
                                                                      &ConstraintSet::getRealSenseDepthMat,
                                                                      this);

  realSenseRGBSubscriber = nh.subscribe<sensor_msgs::ImageConstPtr>("/realsense_camera/image_raw",
                                                                    1,
                                                                    &ConstraintSet::getRealSenseRGBMat, this);

  ros::Rate loop_rate(30);
  while (ros::ok()) {
    if (src_industry_img_.empty()) {
      ROS_INFO("RGB image can't find");
    }
    if (src_realSense_depth_img_.empty()) {
      ROS_INFO("depth image can't find");
    } else {
    }
    if (!src_industry_img_.empty() && !src_realSense_depth_img_.empty()) {
      break;
    } else {
      ROS_INFO("can't get rgb and depth");
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void ConstraintSet::getIndustryMat(sensor_msgs::ImageConstPtr msg) {
  cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(src_industry_img_);
}

void ConstraintSet::getRealSenseDepthMat(sensor_msgs::ImageConstPtr msg) {
  try {
    cv_bridge::toCvShare(msg, "16UC1")->image.copyTo(src_realSense_depth_img_);
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void ConstraintSet::getRealSenseRGBMat(sensor_msgs::ImageConstPtr msg) {
  try {
    cv::Mat src;
    cv_bridge::toCvShare(msg, "bgr8")->image.copyTo(src);
    cv::resize(src, src_realSense_RGB_img_, cv::Size(640, 480));
  } catch (cv_bridge::Exception &e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

//在追踪状态时候是裁剪的ROI，所以有左上角这个参数
ErrorInfo ConstraintSet::SearchArmor(cv::Mat industrialImage,
                                     cv::Mat realSenseRGBImage,
                                     cv::Mat realSenseDepthImage,
                                     cv::Mat imshowImage,
                                     bool &detected,
                                     cv::Point3f &target_3d,
                                     cv::Point2f leftPoint) {
  LightBlobs light_blobs;
  ArmorBoxs armor_boxs;
  std::string classFilePath = ros::package::getPath("roborts_detection") + \
      "/armor_detection/para/";
  Classifier classifier = Classifier(classFilePath);
  if (!industrialImage.empty() && !realSenseRGBImage.empty()) {
    cv::cvtColor(realSenseRGBImage, gray_img_, CV_BGR2GRAY);
    cv::imshow("gray img", gray_img_);
    DetectLights(realSenseRGBImage, light_blobs);
    cv_toolbox_->imshowLightBlobs(imshowImage, light_blobs, "light_blobs", leftPoint);
    PossibleArmors(realSenseRGBImage, light_blobs, armor_boxs);
    cv_toolbox_->imshowArmorBoxs(imshowImage, armor_boxs, "blank", leftPoint);
    ArmorBoxs newArmorBoxs;
    for (ArmorBox armor_box:armor_boxs) {
      //分类器使用工业相机来识别
      cv::Mat resizeClassifier = industrialImage(armor_box.rect).clone();
      cv::resize(resizeClassifier, resizeClassifier, cv::Size(48, 36));
      if ((armor_box.id = classifier(resizeClassifier) != 0)) {
        newArmorBoxs.push_back(armor_box);
      }
    }
    if (newArmorBoxs.size() != 0) {
      detected = true;
      //此时选择第一个识别的装甲板来作为可能的装甲板，由于前面状态机比较麻烦，暂时先记录最可能的装甲板，然后只发一个装甲板坐标消息
      possibleBox = selectFinalArmor(newArmorBoxs);
    } else {
      detected = false;
      possibleBox = ArmorBox();
    }
    cv_toolbox_->imshowArmorBoxs(imshowImage, newArmorBoxs, "classifier", leftPoint);
    //加上裁剪的左上角坐标
    possibleBox.center = possibleBox.center + leftPoint;

    //返回坐标点
    if (detected) {
      ROS_INFO("center x is %lf center y is %lf",
               possibleBox.center.x,
               possibleBox.center.y);
      target_3d.z =
          cv_toolbox_->getDepthByRealSense(src_realSense_depth_img_, possibleBox.center.x, possibleBox.center.y);
      ROS_INFO("get depth %lf", target_3d.z);
      cv_toolbox_->getRealPointByInnerMatrix(possibleBox.center, target_3d, realSenseInnerMatrix);
      ROS_INFO("real x is %lf y is %lf z is %lf ", target_3d.x, target_3d.y, target_3d.z);
    } else {
      target_3d = cv::Point3f(-1, -1, -1);
    }
    return error_info_;
  } else {
    ROS_INFO("can't get image");
  }
}

//跟踪状态
void ConstraintSet::trackingTarget(cv::Mat industrialImage,
                                   cv::Mat realSenseRGBImage,
                                   cv::Mat realSenseDepthImage,
                                   bool &detected,
                                   cv::Point3f &target_3d) {
  auto pos = possibleBox.rect;
  //使用KCFTracker进行跟踪
  if (!tracker->update(industrialImage, pos)) {
    possibleBox = ArmorBox();
    detected = false;
    ROS_INFO("Track fail!");
  }
  if ((pos & cv::Rect2d(0, 0, industrialImage.size().width, industrialImage.size().height)) != pos) {
    possibleBox = ArmorBox();
    detected = false;
    ROS_INFO("Track out range!");
  }
  //此时跟踪没有问题
  //获取相对于追踪区域两倍的矩形面积，进行重新搜索，来获取灯条信息
  cv::Rect2d bigger_rect;
  bigger_rect.x = pos.x - pos.width / 2.0;
  bigger_rect.y = pos.y - pos.height / 2.0;
  bigger_rect.height = pos.height * 2;
  bigger_rect.width = pos.width * 2;
  bigger_rect &= cv::Rect2d(0, 0, 640, 480);
  //进行裁剪
  cv::Mat industrialRoi = industrialImage(bigger_rect).clone();
  cv::Mat realSenseRGBRoi = realSenseRGBImage(bigger_rect).clone();
  //传入左上角坐标
  SearchArmor(industrialRoi,
              realSenseRGBRoi,
              realSenseDepthImage,
              industrialImage,
              detected,
              target_3d,
              bigger_rect.tl());
  //如果两倍区域没有找到，再全局寻找
  if (!detected) {
//    ROS_INFO("22222can't find in two region");
    SearchArmor(industrialImage,
                realSenseRGBImage,
                realSenseDepthImage,
                industrialImage,
                detected,
                target_3d);
  } else {
//    ROS_INFO("333333find in two region");
  }
}

//原框架函数，以后可以删除
ErrorInfo ConstraintSet::DetectArmor(bool &detected, cv::Point3f &target_3d) {
  LightBlobs light_blobs;
  ArmorBoxs armor_boxs;

  auto img_begin = std::chrono::high_resolution_clock::now();
  bool sleep_by_diff_flag = true;
  while (true) {
    // Ensure exit this thread while call Ctrl-C
    if (!thread_running_) {
      ErrorInfo error_info(ErrorCode::STOP_DETECTION);
      return error_info;
    }
    read_index_ = cv_toolbox_->NextImage(src_img_);
    if (read_index_ < 0) {
      // Reducing lock and unlock when accessing function 'NextImage'
      if (detection_time_ == 0) {
        usleep(20000);
        continue;
      } else {
        double capture_time = 0;
        cv_toolbox_->GetCaptureTime(capture_time);
        if (capture_time == 0) {
          // Make sure the driver is launched and the image callback is called
          usleep(20000);
          continue;
        } else if (capture_time > detection_time_ && sleep_by_diff_flag) {
//          ROS_WARN("time sleep %lf", (capture_time - detection_time_));
          usleep((unsigned int) (capture_time - detection_time_));
          sleep_by_diff_flag = false;
          continue;
        } else {
          //For real time request when image call back called, the function 'NextImage' should be called.
          usleep(500);
          continue;
        }
      }
    } else {
      break;
    }
  }
  /*ROS_WARN("time get image: %lf", std::chrono::duration<double, std::ratio<1, 1000>>
      (std::chrono::high_resolution_clock::now() - img_begin).count());*/

  auto detection_begin = std::chrono::high_resolution_clock::now();

  cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
  if (enable_debug_) {
    show_lights_before_filter_ = src_img_.clone();
    show_lights_after_filter_ = src_img_.clone();
    show_armors_befor_filter_ = src_img_.clone();
    show_armors_after_filter_ = src_img_.clone();
    cv::waitKey(1);
  }

  DetectLights(src_img_, light_blobs);
  PossibleArmors(src_img_, light_blobs, armor_boxs);
//  FilterArmors(armors);
  if (!armor_boxs.empty()) {
    detected = true;
    //下面三行暂时注释 暂未实现下面两个函数
//    ArmorInfo final_armor = selectFinalArmor(armors);
//    cv_toolbox_->DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
//    CalcControlInfo(final_armor, target_3d);
  } else
    detected = false;
  if (enable_debug_) {
    cv::imshow("relust_img_", src_img_);
  }

  light_blobs.clear();
  armor_boxs.clear();
  cv_toolbox_->ReadComplete(read_index_);
  ROS_INFO("read complete");
  detection_time_ = std::chrono::duration<double, std::ratio<1, 1000000>>
      (std::chrono::high_resolution_clock::now() - detection_begin).count();

  return error_info_;
}

void ConstraintSet::DetectLights(cv::Mat &src, LightBlobs &light_blobs) {
  //std::cout << "********************************************DetectLights********************************************" << std::endl;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(src, src, element, cv::Point(-1, -1), 1);
  cv::Mat binary_color_img;
  if (using_hsv_) {
    binary_color_img = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
  } else {
    cv::Mat rgb_channel = cv_toolbox_->DistillationColor(src, enemy_color_, using_hsv_);
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;
    cv::threshold(rgb_channel, binary_color_img, thresh, 255, CV_THRESH_BINARY);
  }
  if (enable_debug_) {
    cv::imshow("binary_color_img", binary_color_img);
  } else {
    ROS_INFO("debug can not");
  }
  //在realSense彩图中，曝光比较高，用于识别灯管，再进行膨胀
  cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);
  std::vector<std::vector<cv::Point>> light_contours;
  std::vector<cv::Vec4i> hierarchy;
  findContours(binary_color_img, light_contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
  std::vector<cv::RotatedRect> rotatedRects;
  for (int i = 0; i < light_contours.size(); i++) {
    //此时没有父轮廓，是最大的轮廓
    if (hierarchy[i][2] == -1) {
      cv::RotatedRect rect = minAreaRect(light_contours[i]);
      if (LightBlob::isValidLightBlob(light_contours[i], rect)) {
        rotatedRects.push_back(rect);
        if (cv_toolbox_->get_rect_color(binary_color_img, rect) != -1) {
          light_blobs.emplace_back(rect,
                                   LightBlob::areaRatio(light_contours[i], rect),
                                   cv_toolbox_->get_rect_color(src, rect));
        }
      }
    }
  }

  auto c = cv::waitKey(1);
  if (c == 'a') {
    cv::waitKey(0);
  }
}

//匹配可能的装甲板
void ConstraintSet::PossibleArmors(cv::Mat &src, LightBlobs &lightBlobs, ArmorBoxs &armor_boxs) {
  cv::Mat result_pic_blank = src.clone();
  LightBlobs lightBlobsTemp;
  lightBlobsTemp.swap(lightBlobs);
  for (int i = 0; i < lightBlobsTemp.size(); i++) {
    for (int j = i + 1; j < lightBlobsTemp.size(); j++) {
      if (!ArmorBox::isCoupleLight(lightBlobsTemp.at(i), lightBlobsTemp.at(j), enemy_color_)) {
        continue;
      }
      cv::Rect2d rect_i = lightBlobsTemp.at(static_cast<unsigned long>(i)).rect.boundingRect();
      cv::Rect2d rect_j = lightBlobsTemp.at(static_cast<unsigned long>(j)).rect.boundingRect();
      double min_x, max_x, min_y, max_y;
      min_x = fmin(rect_i.x, rect_j.x);
      max_x = fmax(rect_i.x + rect_i.width, rect_j.x + rect_j.width);
      min_y = fmin(rect_i.y, rect_j.y);
      max_y = fmax(rect_i.y + rect_i.height, rect_j.y + rect_j.height);
      double interpolation_y = max_y - min_y;
      //扩大y的范围
      min_y = min_y - interpolation_y / 2;
      max_y = max_y + interpolation_y / 2;
      if (min_x < 0 || max_x > src.cols || min_y < 0 || max_y > src.rows) {
        continue;
      }
      lightBlobs.push_back(lightBlobsTemp.at(i));
      lightBlobs.push_back(lightBlobsTemp.at(j));
      LightBlobs pair_blobs = {lightBlobsTemp.at(i), lightBlobsTemp.at(j)};
      ArmorBox armor_box = ArmorBox(cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y), pair_blobs, enemy_color_);
      if (armor_box.isMatchArmorBox()) {
        armor_boxs.emplace_back(armor_box);
      }
    }
  }
  cv_toolbox_->imshowArmorBoxs(src, armor_boxs, "blank");
}

//原框架函数，以后可以删除
void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
  //std::cout << "********************************************FilterArmors********************************************" << std::endl;
  cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
    cv::Point pts[4];
    for (unsigned int i = 0; i < 4; i++) {
      pts[i].x = (int) armor_iter->vertex[i].x;
      pts[i].y = (int) armor_iter->vertex[i].y;
    }
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);

    cv::Mat mat_mean;
    cv::Mat mat_stddev;
    cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);

    auto stddev = mat_stddev.at<double>(0, 0);
    auto mean = mat_mean.at<double>(0, 0);
    //std::cout << "stddev: " << stddev << std::endl;
    //std::cout << "mean: " << mean << std::endl;

    if (stddev > armor_max_stddev_ || mean > armor_max_mean_) {
      armor_iter = armors.erase(armor_iter);
    } else {
      armor_iter++;
    }
  }

  // nms
  std::vector<bool> is_armor(armors.size(), true);
  for (int i = 0; i < armors.size() && is_armor[i] == true; i++) {
    for (int j = i + 1; j < armors.size() && is_armor[j]; j++) {
      float dx = armors[i].rect.center.x - armors[j].rect.center.x;
      float dy = armors[i].rect.center.y - armors[j].rect.center.y;
      float dis = std::sqrt(dx * dx + dy * dy);
      if (dis < armors[i].rect.size.width + armors[j].rect.size.width) {
        if (armors[i].rect.angle > armors[j].rect.angle) {
          is_armor[i] = false;
          //std::cout << "i: " << i << std::endl;
        } else {
          is_armor[j] = false;
          //std::cout << "j: " << j << std::endl;
        }
      }
    }
  }
  //std::cout << armors.size() << std::endl;
  for (unsigned int i = 0; i < armors.size(); i++) {
    if (!is_armor[i]) {
      armors.erase(armors.begin() + i);
      is_armor.erase(is_armor.begin() + i);
      //std::cout << "index: " << i << std::endl;
    } else if (enable_debug_) {
      cv_toolbox_->DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
    }
  }
  if (enable_debug_)
    cv::imshow("armors_after_filter", show_armors_after_filter_);
}

//之前想加一个过滤装甲板调函数，用于过滤如何两个装甲板有相交的情况，后面发现实际效果不是很好
void ConstraintSet::FilterArmors(cv::Mat &src, ArmorBoxs &armor_boxs) {
  cv::Mat result_pic_filter_armors = src.clone();
  ArmorBoxs armorBoxsTemp;
  armorBoxsTemp.swap(armor_boxs);
  std::vector<int> needRemoveArmor;
  ROS_INFO("test filter size%d", armorBoxsTemp.size());
  for (int i = 0; i < armorBoxsTemp.size(); i++) {
    for (int j = i + 1; j < armorBoxsTemp.size(); j++) {
      float dx = armorBoxsTemp[i].center.x - armorBoxsTemp[j].center.x;
      float dy = armorBoxsTemp[i].center.y - armorBoxsTemp[j].center.y;
      float dis = std::sqrt(dx * dx + dy * dy);
      //如果两个装甲板相交，则去除较大的
      //乘1.2避免边线相交情况
      if (dis * 1.2 < std::min(armorBoxsTemp[i].rect.width + armorBoxsTemp[j].rect.width,
                               armorBoxsTemp[i].rect.height + armorBoxsTemp[j].rect.height)) {
        bool flag = true;
        if (armorBoxsTemp[i].rect.height * armorBoxsTemp[i].rect.width <
            armorBoxsTemp[j].rect.height * armorBoxsTemp[j].rect.width) {
          for (int t = 0; t < needRemoveArmor.size(); t++) {
            if (needRemoveArmor[t] == j) {
              flag = false;
              break;
            }
          }
          if (flag) {
            needRemoveArmor.push_back(j);
          }
        } else {
          for (int t = 0; t < needRemoveArmor.size(); t++) {
            if (needRemoveArmor[t] == i) {
              flag = false;
              break;
            }
          }
          if (flag) {
            needRemoveArmor.push_back(i);
          }
        }
      }
    }
    std::sort(needRemoveArmor.begin(), needRemoveArmor.end());
    int t = 0;
    for (int i = 0; i < armorBoxsTemp.size(); i++) {
      if (t >= needRemoveArmor.size() || i != needRemoveArmor[t]) {
        armor_boxs.push_back(armorBoxsTemp[i]);
      } else {
        t++;
      }
    }
  }
  cv_toolbox_->imshowArmorBoxs(src, armor_boxs, "filer armors");
}

//ArmorInfo ConstraintSet::selectFinalArmor(std::vector<ArmorInfo> &armors) {
//  std::sort(armors.begin(),
//            armors.end(),
//            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });
//
//  return armors[0];
//}


//前期为了方便，只选择了尺寸最大的装甲板
ArmorBox ConstraintSet::selectFinalArmor(ArmorBoxs armorBoxs) {
  std::sort(armorBoxs.begin(),
            armorBoxs.end(),
            [](const ArmorBox &p1, const ArmorBox &p2) { return p1.rect.area() > p2.rect.area(); });

  return armorBoxs[0];
}

void ConstraintSet::CalcControlInfo(const ArmorInfo &armor, cv::Point3f &target_3d) {
  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(armor_points_,
               armor.vertex,
               intrinsic_matrix_,
               distortion_coeffs_,
               rvec,
               tvec);
  target_3d = cv::Point3f(tvec);

}

void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
                                  cv::RotatedRect left_light,
                                  cv::RotatedRect right_light) {
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);

  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);

}

void ConstraintSet::SolveArmorCoordinate(const float width,
                                         const float height) {
  armor_points_.emplace_back(cv::Point3f(-width / 2, height / 2, 0.0));
  armor_points_.emplace_back(cv::Point3f(width / 2, height / 2, 0.0));
  armor_points_.emplace_back(cv::Point3f(width / 2, -height / 2, 0.0));
  armor_points_.emplace_back(cv::Point3f(-width / 2, -height / 2, 0.0));
}

void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
  if (fabs(new_num - old_num) > max_diff && filter_count < 2) {
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}

void ConstraintSet::SetThreadState(bool thread_state) {
  thread_running_ = thread_state;
}

ConstraintSet::~ConstraintSet() {

}
} //namespace roborts_detection
