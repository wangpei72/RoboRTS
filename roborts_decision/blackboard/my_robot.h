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
#include "../executor/gimbal_executor.h"
#include "../executor/chassis_executor.h"

namespace roborts_decision {

class MyRobot {
 public:

  explicit MyRobot(RobotId id, const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~MyRobot();

  RobotId GetId() const;

  int GetHp() const;

  int GetCurrentHeat() const;

  int GetRemainingProjectiles() const;

  bool IsSurvival() const;
  void SetIsSurvival(bool is_survival);

  const std::vector<ArmorId> &GetArmorsUnderAttack() const;

  const roborts_msgs::ArmorsDetected &GetArmorsInEyes() const;

  const geometry_msgs::PoseStamped &GetChassisMapPose() const;

  const geometry_msgs::PoseStamped &GetChassisOdomPose() const;

  const geometry_msgs::PoseStamped &GetGimbalMapPose() const;

  const geometry_msgs::PoseStamped &GetGimbalOdomPose() const;

  const geometry_msgs::PoseStamped &GetCurrentGoal() const;

  MyRobotBehavior GetCurrentBehavior() const;
  void SetCurrentBehavior(MyRobotBehavior current_behavior);

  const ChassisExecutor &GetChassisExecutor() const;

  const GimbalExecutor &GetGimbalExecutor() const;

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
  bool is_survival_;

  std::vector<ArmorId> armors_under_attack_;
  roborts_msgs::ArmorsDetected armors_in_eyes_;

  geometry_msgs::PoseStamped chassis_map_pose_;
  geometry_msgs::PoseStamped chassis_odom_pose_;

  geometry_msgs::PoseStamped gimbal_map_pose_;
  geometry_msgs::PoseStamped gimbal_odom_pose_;

  geometry_msgs::PoseStamped current_goal_;
  MyRobotBehavior current_behavior_;

  ChassisExecutor chassis_executor_;
  GimbalExecutor gimbal_executor_;
};

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
