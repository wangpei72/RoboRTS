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

#include <cv_bridge/cv_bridge.h>

#include "camera_node.h"

namespace roborts_camera{
CameraNode::CameraNode(){
  camera_num_ = camera_param_.GetCameraParam().size();
  img_pubs_.resize(camera_num_);
    depth_pubs_.resize(camera_num_);
  camera_threads_.resize(camera_num_);
  camera_driver_.resize(camera_num_);

  for (unsigned int i = 0; i < camera_num_; i++) {
    auto camera_info = camera_param_.GetCameraParam()[i];
    nhs_.push_back(ros::NodeHandle(camera_info.camera_name));
    image_transport::ImageTransport it(nhs_.at(i));
    img_pubs_[i] = it.advertiseCamera("image_raw", 1, true);

      if (camera_info.depth_enable) {
          depth_pubs_[i] = it.advertise("depth_raw", 1, true);
          depth_pubs_convert_[i] = it.advertise("depth_convert", 1, true);
      } else {

      }
    //create the selected camera driver
    camera_driver_[i] = roborts_common::AlgorithmFactory<CameraBase,CameraInfo>::CreateAlgorithm(camera_info.camera_type,camera_info);
  }

  StartThread();
}

void CameraNode::StartThread() {
  running_ = true;
  for (unsigned int i = 0; i < camera_num_; i++) {
    camera_threads_[i] = std::thread(&CameraNode::Update, this, i);
  }
}

void CameraNode::Update(const unsigned int index) {
    cv::Mat img, depth;
  bool camera_info_send = false;
  while(running_) {
      camera_driver_[index]->StartReadCamera(img, depth);
      if (!img.empty()) {


          sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                             camera_param_.GetCameraParam()[index].image_code,
                                                             img).toImageMsg();
          img_msg->header.frame_id = camera_param_.GetCameraParam()[index].camera_name;
          img_msg->header.stamp = ros::Time::now();

          camera_param_.GetCameraParam()[index].ros_camera_info->header.stamp = img_msg->header.stamp;
          img_pubs_[index].publish(img_msg, camera_param_.GetCameraParam()[index].ros_camera_info);
      }

      if (camera_param_.GetCameraParam()[index].depth_enable && !depth.empty()) {
          sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(),
                                                               camera_param_.GetCameraParam()[index].depth_code,
                                                               depth).toImageMsg();

          depth_msg->header.frame_id = camera_param_.GetCameraParam()[index].camera_name;
          depth_msg->header.stamp = ros::Time::now();

          //ROS_INFO("time: %d ",ros::Time::now().nsec);

          camera_param_.GetCameraParam()[index].ros_camera_info->header.stamp = depth_msg->header.stamp;
          camera_convert convert = camera_convert(depth);
          convert.pixel_points_ = convert.get_pixel_points_(depth);
          convert.img_depth_dst_ = convert.get_depth_dst_();
          sensor_msgs::ImagePtr depth_msg_convert = cv_bridge::CvImage(std_msgs::Header(),
                                                                       camera_param_.GetCameraParam()[index].depth_code,
                                                                       convert.img_depth_dst_
          ).toImageMsg();
          depth_pubs_[index].publish(depth_msg);
          depth_pubs_convert_[index].publish(depth_msg_convert);
          //ROS_INFO("depth publish");
      }



    else {
        ROS_ERROR("%s : %s fail to publish ",
                camera_param_.GetCameraParam()[index].camera_type.c_str(),
                camera_param_.GetCameraParam()[index].camera_name.c_str());
    }
  }
}

void CameraNode::StoptThread() {
//TODO: To be implemented
}

CameraNode::~CameraNode() {
  running_ = false;
  for (auto &iter: camera_threads_) {
    if (iter.joinable())
      iter.join();
  }
}
} //namespace roborts_camera

void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

int main(int argc, char **argv){
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "roborts_camera_node", ros::init_options::NoSigintHandler);
  roborts_camera::CameraNode camera_test;
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
