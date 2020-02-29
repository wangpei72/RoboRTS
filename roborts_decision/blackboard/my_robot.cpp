//
// Created by kehan on 2020/2/28.
//

#include "my_robot.h"
using namespace roborts_decision;

MyRobot::MyRobot(const ros::NodeHandle &nh) :
    nh_(nh),
    hp_(2000),
    current_heat_(0),
    is_survival_(true),
    remaining_projectiles_(0) {

  id_ = RobotId::UNKNOWN;
//  if (id_ == RED1 || id_ == BLUE1) {
//    remaining_projectiles_ = 50;
//  } else {
//    remaining_projectiles_ = 0;
//  }

  // TODO
  current_behavior_ = MyRobotBehavior::SEARCH;
}

MyRobot::~MyRobot() = default;

RobotId MyRobot::GetId() const {
  return id_;
}

int MyRobot::GetHp() const {
  return hp_;
}

int MyRobot::GetCurrentHeat() const {
  return current_heat_;
}

int MyRobot::GetRemainingProjectiles() const {
  return remaining_projectiles_;
}

bool MyRobot::IsSurvival() const {
  return is_survival_;
}

void MyRobot::SetIsSurvival(bool is_survival) {
  is_survival_ = is_survival;
}

const std::vector<ArmorId> &MyRobot::GetArmorsUnderAttack() const {
  return armors_under_attack_;
}

const roborts_msgs::ArmorsDetected &MyRobot::GetArmorsInEyes() const {
  return armors_in_eyes_;
}

const geometry_msgs::PoseStamped &MyRobot::GetChassisMapPose() const {
  return chassis_map_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::GetChassisOdomPose() const {
  return chassis_odom_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::GetGimbalMapPose() const {
  return gimbal_map_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::GetGimbalOdomPose() const {
  return gimbal_odom_pose_;
}

const geometry_msgs::PoseStamped &MyRobot::GetCurrentGoal() const {
  return current_goal_;
}

MyRobotBehavior MyRobot::GetCurrentBehavior() const {
  return current_behavior_;
}

void MyRobot::SetCurrentBehavior(MyRobotBehavior current_behavior) {
  current_behavior_ = current_behavior;
}

bool MyRobot::operator==(const MyRobot &rhs) const {
  return id_ == rhs.id_;
}

bool MyRobot::operator!=(const MyRobot &rhs) const {
  return !(rhs == *this);
}


