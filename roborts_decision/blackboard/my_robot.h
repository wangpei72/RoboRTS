//
// Created by kehan on 2020/2/28.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_

#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/ArmorsDetected.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#include "blackboard_common.h"

namespace roborts_decision {

class MyRobot {

 private:

  std::shared_ptr<tf::TransformListener> tf_ptr_;

  int id_;
  int hp_;
  int remaining_projectiles_;
  std::vector<ArmorId> armors_under_attack_;
  roborts_msgs::ArmorsDetected armors_in_eyes_;

  geometry_msgs::PoseStamped chassis_map_pose_;
  geometry_msgs::PoseStamped chassis_odom_pose_;

  geometry_msgs::PoseStamped gimbal_map_pose_;
  geometry_msgs::PoseStamped gimbal_odom_pose_;

  RmRobotBehavior current_behavior_;
  geometry_msgs::PoseStamped current_goal_;

};

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
