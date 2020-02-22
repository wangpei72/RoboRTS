//
// Created by heihei on 2019/12/29.
//

#include "pose_publisher_node.h"
int main(int argc, char **argv) {

  ros::init(argc, argv, "pose_publisher_node");

  PosePublisher pose_publisher;
  ros::Rate rate(100);

  while (ros::ok()) {
    pose_publisher.Update();
    rate.sleep();
  }

  return 0;
}