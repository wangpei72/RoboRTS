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

bool roborts_decision::EnemyRobot::IsSurvival() const {
  return is_survival_;
}

void roborts_decision::EnemyRobot::SetIsSurvival(bool is_survival) {
  is_survival_ = is_survival;
}

const geometry_msgs::PoseStamped &roborts_decision::EnemyRobot::GetPose() const {
  return pose_;
}



