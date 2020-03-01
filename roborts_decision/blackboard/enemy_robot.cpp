//
// Created by kehan on 2020/2/28.
//

#include "enemy_robot.h"

roborts_decision::EnemyRobot::EnemyRobot(const RobotId &robot_id) :
    id_(robot_id),
    is_survival_(true) {

}

roborts_decision::EnemyRobot::~EnemyRobot() = default;

roborts_decision::RobotId roborts_decision::EnemyRobot::GetId() const {
  return id_;
}

roborts_decision::RobotType roborts_decision::EnemyRobot::GetRobotType() const {
  return robot_type_;
}

void roborts_decision::EnemyRobot::SetRobotType(roborts_decision::RobotType robot_type) {
  robot_type_ = robot_type;
}

bool roborts_decision::EnemyRobot::IsSurvival() const {
  return is_survival_;
}

void roborts_decision::EnemyRobot::SetIsSurvival(bool is_survival) {
  is_survival_ = is_survival;
}

const geometry_msgs::PoseStamped &roborts_decision::EnemyRobot::GetPose() const {
  return pose_;
}

void roborts_decision::EnemyRobot::SetPose(const ros::Time &stamp, const geometry_msgs::Point &position) {
  pose_.header.stamp = stamp;
  pose_.pose.position = position;
}

bool roborts_decision::EnemyRobot::IsDetected() const {
  return is_detected_;
}

void roborts_decision::EnemyRobot::SetIsDetected(bool is_detected) {
  is_detected_ = is_detected;
}




