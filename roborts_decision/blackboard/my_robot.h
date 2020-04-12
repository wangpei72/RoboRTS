//
// Created by kehan on 2020/2/28.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_

#include <vector>
#include <thread>
#include <mutex>
#include <geometry_msgs/PoseStamped.h>

#include <roborts_msgs/ArmorsDetected.h>
#include <roborts_msgs/RobotStatus.h>

#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <roborts_msgs/RobotHeat.h>
#include <roborts_msgs/RobotDamage.h>

#include "blackboard_common.h"
#include "../executor/gimbal_executor.h"
#include "../executor/chassis_executor.h"

namespace roborts_decision {

class MyRobot {
 public:

  explicit MyRobot(RobotId id, const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~MyRobot();

  RobotId GetId() const;

  RobotType GetRobotType() const;
  void SetRobotType(RobotType robot_type);

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

  // const geometry_msgs::PoseStamped &GetCurrentGoal() const;

  MyRobotBehavior GetCurrentBehavior() const;
  void SetCurrentBehavior(MyRobotBehavior current_behavior);

  const ChassisExecutor &GetChassisExecutor();

  const GimbalExecutor &GetGimbalExecutor();

  bool operator==(const MyRobot &rhs) const;
  bool operator!=(const MyRobot &rhs) const;

 private:
  void ArmorsUnderAttackCallback(const roborts_msgs::RobotDamage::ConstPtr &msg);
  void HeatCallback(const roborts_msgs::RobotHeat::ConstPtr &msg);
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg);
  void ArmorsInEyesCallback(const roborts_msgs::ArmorsDetected::ConstPtr &msg);

  void UpdateChassisMapPose();
  void UpdateChassisOdomPose();
  void UpdateGimbalMapPose();
  void UpdateGimbalOdomPose();

  ros::NodeHandle nh_;

  ros::Subscriber armors_under_attack_sub_;
  ros::Subscriber heat_sub_;
  ros::Subscriber armors_in_eyes_sub_;
  ros::Subscriber robot_status_sub_;

  std::shared_ptr<tf::TransformListener> tf_ptr_;
  std::shared_ptr<std::thread> tf_thread_ptr_;

  RobotId id_;
  RobotType robot_type_;

  int remaining_hp_;
  int max_hp_;
  int current_heat_;

  // TODO
  int remaining_projectiles_;

  bool is_survival_;

  std::vector<ArmorId> armors_under_attack_;
  roborts_msgs::ArmorsDetected armors_in_eyes_;

  geometry_msgs::PoseStamped chassis_map_pose_;
  geometry_msgs::PoseStamped chassis_odom_pose_;

  geometry_msgs::PoseStamped gimbal_map_pose_;
  geometry_msgs::PoseStamped gimbal_odom_pose_;

  // geometry_msgs::PoseStamped current_goal_;
  MyRobotBehavior current_behavior_;

  ChassisExecutor chassis_executor_;
  GimbalExecutor gimbal_executor_;
};

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_RMROBOT_H_
