//
// Created by kehan on 2020/2/28.
//

#include "my_robot.h"
using namespace roborts_decision;

MyRobot::MyRobot(const ros::NodeHandle &nh) {

}

MyRobot::~MyRobot() {

}

RobotId MyRobot::GetId() const {
  return id_;
}

void MyRobot::SetId(RobotId id) {
  id_ = id;
}

int MyRobot::GetHp() const {
  return hp_;
}

void MyRobot::SetHp(int hp) {
  hp_ = hp;
}

int MyRobot::GetCurrentHeat() const {
  return current_heat_;
}

void MyRobot::SetCurrentHeat(int current_heat) {
  current_heat_ = current_heat;
}

int MyRobot::GetRemainingProjectiles() const {
  return remaining_projectiles_;
}

void MyRobot::SetRemainingProjectiles(int remaining_projectiles) {
  remaining_projectiles_ = remaining_projectiles;
}

const std::vector<ArmorId> &MyRobot::GetArmorsUnderAttack() const {
  return armors_under_attack_;
}

void MyRobot::SetArmorsUnderAttack(const std::vector<ArmorId> &armors_under_attack) {
  armors_under_attack_ = armors_under_attack;
}

const roborts_msgs::ArmorsDetected &MyRobot::GetArmorsInEyes() const {
  return armors_in_eyes_;
}

void MyRobot::SetArmorsInEyes(const roborts_msgs::ArmorsDetected &armors_in_eyes) {
  armors_in_eyes_ = armors_in_eyes;
}

const geometry_msgs::PoseStamped &MyRobot::GetChassisMapPose() const {
  return chassis_map_pose_;
}

void MyRobot::SetChassisMapPose(const geometry_msgs::PoseStamped &chassis_map_pose) {
  chassis_map_pose_ = chassis_map_pose;
}

const geometry_msgs::PoseStamped &MyRobot::GetChassisOdomPose() const {
  return chassis_odom_pose_;
}

void MyRobot::SetChassisOdomPose(const geometry_msgs::PoseStamped &chassis_odom_pose) {
  chassis_odom_pose_ = chassis_odom_pose;
}

const geometry_msgs::PoseStamped &MyRobot::GetGimbalMapPose() const {
  return gimbal_map_pose_;
}

void MyRobot::SetGimbalMapPose(const geometry_msgs::PoseStamped &gimbal_map_pose) {
  gimbal_map_pose_ = gimbal_map_pose;
}

const geometry_msgs::PoseStamped &MyRobot::GetGimbalOdomPose() const {
  return gimbal_odom_pose_;
}

void MyRobot::SetGimbalOdomPose(const geometry_msgs::PoseStamped &gimbal_odom_pose) {
  gimbal_odom_pose_ = gimbal_odom_pose;
}

const geometry_msgs::PoseStamped &MyRobot::GetCurrentGoal() const {
  return current_goal_;
}

void MyRobot::SetCurrentGoal(const geometry_msgs::PoseStamped &current_goal) {
  current_goal_ = current_goal;
}

RmRobotBehavior MyRobot::GetCurrentBehavior() const {
  return current_behavior_;
}

void MyRobot::SetCurrentBehavior(RmRobotBehavior current_behavior) {
  current_behavior_ = current_behavior;
}

bool MyRobot::operator==(const MyRobot &rhs) const {

  if (!IsPoseMsgEqual(chassis_map_pose_, rhs.chassis_map_pose_)) {
    return false;
  }

  if (!IsPoseMsgEqual(chassis_odom_pose_, rhs.chassis_odom_pose_)) {
    return false;
  }

  if (!IsPoseMsgEqual(gimbal_map_pose_, rhs.gimbal_map_pose_)) {
    return false;
  }

  if (!IsPoseMsgEqual(gimbal_odom_pose_, rhs.gimbal_odom_pose_)) {
    return false;
  }

  if (!IsPoseMsgEqual(current_goal_, rhs.current_goal_)) {
    return false;
  }

  return id_ == rhs.id_ &&
      hp_ == rhs.hp_ &&
      current_heat_ == rhs.current_heat_ &&
      remaining_projectiles_ == rhs.remaining_projectiles_ &&
      armors_under_attack_ == rhs.armors_under_attack_ &&
      armors_in_eyes_ == rhs.armors_in_eyes_ &&
      current_behavior_ == rhs.current_behavior_;
}

bool MyRobot::operator!=(const MyRobot &rhs) const {
  return !(rhs == *this);
}
