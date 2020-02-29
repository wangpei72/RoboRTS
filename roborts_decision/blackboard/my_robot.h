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
 public:

  explicit MyRobot(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~MyRobot();

  RobotId GetId() const;
  void SetId(RobotId id);

  int GetHp() const;
  void SetHp(int hp);

  int GetCurrentHeat() const;
  void SetCurrentHeat(int current_heat);

  int GetRemainingProjectiles() const;
  void SetRemainingProjectiles(int remaining_projectiles);

  const std::vector<ArmorId> &GetArmorsUnderAttack() const;
  void SetArmorsUnderAttack(const std::vector<ArmorId> &armors_under_attack);

  const roborts_msgs::ArmorsDetected &GetArmorsInEyes() const;
  void SetArmorsInEyes(const roborts_msgs::ArmorsDetected &armors_in_eyes);

  const geometry_msgs::PoseStamped &GetChassisMapPose() const;
  void SetChassisMapPose(const geometry_msgs::PoseStamped &chassis_map_pose);

  const geometry_msgs::PoseStamped &GetChassisOdomPose() const;
  void SetChassisOdomPose(const geometry_msgs::PoseStamped &chassis_odom_pose);

  const geometry_msgs::PoseStamped &GetGimbalMapPose() const;
  void SetGimbalMapPose(const geometry_msgs::PoseStamped &gimbal_map_pose);

  const geometry_msgs::PoseStamped &GetGimbalOdomPose() const;
  void SetGimbalOdomPose(const geometry_msgs::PoseStamped &gimbal_odom_pose);

  const geometry_msgs::PoseStamped &GetCurrentGoal() const;
  void SetCurrentGoal(const geometry_msgs::PoseStamped &current_goal);

  RmRobotBehavior GetCurrentBehavior() const;
  void SetCurrentBehavior(RmRobotBehavior current_behavior);

  bool operator==(const MyRobot &rhs) const;
  bool operator!=(const MyRobot &rhs) const;

 private:

  ros::NodeHandle nh_;
  ros::Subscriber armors_under_attack_sub_;
  ros::Subscriber armors_in_eyes_sub_;

  std::shared_ptr<tf::TransformListener> tf_ptr_;

  RobotId id_;
  int hp_;
  int current_heat_;
  int remaining_projectiles_;

  std::vector<ArmorId> armors_under_attack_;
  roborts_msgs::ArmorsDetected armors_in_eyes_;

  geometry_msgs::PoseStamped chassis_map_pose_;
  geometry_msgs::PoseStamped chassis_odom_pose_;

  geometry_msgs::PoseStamped gimbal_map_pose_;
  geometry_msgs::PoseStamped gimbal_odom_pose_;

  geometry_msgs::PoseStamped current_goal_;
  RmRobotBehavior current_behavior_;

};

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
