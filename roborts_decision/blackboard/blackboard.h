//
// Created by kehan on 2020/3/1.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_

#include <ros/ros.h>
#include <roborts_msgs/RobotStatus.h>

#include "roborts_msgs/OutpostDetected.h"
#include "roborts_msgs/BuffZoneStatus.h"
#include "roborts_msgs/GameSurvivor.h"

#include "my_robot.h"
#include "enemy_robot.h"
#include "blackboard_common.h"

namespace roborts_decision {
class Blackboard {
 public:
  explicit Blackboard(std::shared_ptr<MyRobot> &p_myrobot1,
                      std::shared_ptr<MyRobot> &p_myrobot2,
                      const ros::NodeHandle &nh = ros::NodeHandle("~"));
  virtual ~Blackboard();

  const std::shared_ptr<MyRobot> &GetMyRobot1();
  const std::shared_ptr<MyRobot> &GetMyRobot2();

  const EnemyRobot &GetEnemyRobot1() const;
  const EnemyRobot &GetEnemyRobot2() const;
  /*
   * .at(1) means F1 , ... , .at(6) means F6
   */
  const std::vector<BuffZoneStatus> &GetBuffZoneStatus() const;

  MyColor GetMyColor() const;

 private:
  void OutpostDetectedCallback(const roborts_msgs::OutpostDetected::ConstPtr &msg);
  void BuffZoneStatusCallback(const roborts_msgs::BuffZoneStatus::ConstPtr &msg);
  void MyColorCallback(const roborts_msgs::RobotStatus::ConstPtr &msg);
  void GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr &msg);
  void Blue1GroundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg);
  void Blue2GroundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg);

  ros::NodeHandle nh_;
  ros::Subscriber outpose_camera_sub_;
  ros::Subscriber buff_zone_sub_;
  ros::Subscriber color_sub_;
  ros::Subscriber game_survivor_sub_;
  ros::Subscriber blue1_gt_sub_;
  ros::Subscriber blue2_gt_sub_;

  std::vector<BuffZoneStatus> vec_buff_zone_status_;

  MyColor my_color_;

  EnemyRobot enemy_robot_1_;
  EnemyRobot enemy_robot_2_;

  std::shared_ptr<MyRobot> p_my_robot1_;
  std::shared_ptr<MyRobot> p_my_robot2_;
};
}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_H_
