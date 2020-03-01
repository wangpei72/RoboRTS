//
// Created by kehan on 2020/3/1.
//

#include "blackboard.h"
using namespace roborts_decision;

Blackboard::Blackboard(const ros::NodeHandle &nh)
    : nh_(nh),
      my_robot_1_(RobotId::MY_ROBOT_1, ros::NodeHandle("/my_robot_1")),
      my_robot_2_(RobotId::MY_ROBOT_2, ros::NodeHandle("/my_robot_2")),
      enemy_robot_1_(RobotId::ENEMY_ROBOT_1),
      enemy_robot_2_(RobotId::ENEMY_ROBOT_2) {

}

const MyRobot &Blackboard::GetMyRobot1() const {
  return my_robot_1_;
}

const MyRobot &Blackboard::GetMyRobot2() const {
  return my_robot_2_;
}

const EnemyRobot &Blackboard::GetEnemyRobot1() const {
  return enemy_robot_1_;
}

const EnemyRobot &Blackboard::GetEnemyRobot2() const {
  return enemy_robot_2_;
}

const std::vector<BuffZoneStatus> &Blackboard::GetBuffZoneStatus() const {
  return buff_zone_status_;
}


