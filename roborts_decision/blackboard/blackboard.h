//
// Created by kehan on 2020/3/1.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_

#include <ros/ros.h>
#include "my_robot.h"
#include "enemy_robot.h"
#include "blackboard_common.h"

namespace roborts_decision {
class Blackboard {
 public:
  explicit Blackboard(const ros::NodeHandle &nh = ros::NodeHandle("~"));
  const MyRobot &GetMyRobot1() const;
  const MyRobot &GetMyRobot2() const;
  const EnemyRobot &GetEnemyRobot1() const;
  const EnemyRobot &GetEnemyRobot2() const;
  const std::vector<BuffZoneStatus> &GetBuffZoneStatus() const;

 private:
  ros::NodeHandle nh_;
  ros::Subscriber outpose_camera_sub_;

  std::vector<BuffZoneStatus> buff_zone_status_;

  MyRobot my_robot_1_;
  MyRobot my_robot_2_;
  EnemyRobot enemy_robot_1_;
  EnemyRobot enemy_robot_2_;
};
}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_
