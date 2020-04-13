//
// Created by kehan on 2020/3/1.
//

#include "blackboard.h"
using namespace roborts_decision;

Blackboard::Blackboard(std::shared_ptr<MyRobot> &p_myrobot1,
                       std::shared_ptr<MyRobot> &p_myrobot2,
                       const ros::NodeHandle &nh) :
    nh_(nh),
    enemy_robot_1_(RobotId::ENEMY_ROBOT_1),
    enemy_robot_2_(RobotId::ENEMY_ROBOT_2),
    my_color_(UNKNOWN_COLOR),
    p_my_robot1_(p_myrobot1),
    p_my_robot2_(p_myrobot2) {

  std::string outpost_camera_topic("/outpost_camera");
  nh_.param("outpost_camera_topic", outpost_camera_topic, outpost_camera_topic);
  outpose_camera_sub_ =
      nh_.subscribe<roborts_msgs::OutpostDetected>(outpost_camera_topic, 1, &Blackboard::OutpostDetectedCallback, this);

  std::string buff_zone_topic("/buff_zone_status");
  nh_.param("buff_zone_status_topic", buff_zone_topic, buff_zone_topic);
  buff_zone_sub_ =
      nh_.subscribe<roborts_msgs::BuffZoneStatus>(buff_zone_topic, 1, &Blackboard::BuffZoneStatusCallback, this);

  std::string robot_status_for_color_topic("/robot_status_for_color");
  nh_.param("robot_status_for_color_topic", robot_status_for_color_topic, robot_status_for_color_topic);
  color_sub_ =
      nh_.subscribe<roborts_msgs::RobotStatus>(robot_status_for_color_topic, 1, &Blackboard::MyColorCallback, this);

  std::string game_survivor_topic("/game_survivor");
  nh_.param("game_survivor_topic", game_survivor_topic, game_survivor_topic);
  game_survivor_sub_ =
      nh_.subscribe<roborts_msgs::GameSurvivor>(game_survivor_topic, 1, &Blackboard::GameSurvivorCallback, this);

  blue1_gt_sub_ =
      nh_.subscribe<nav_msgs::Odometry>("/blue1/ground_truth/state", 1, &Blackboard::Blue1GroundTruthCallback, this);
  blue2_gt_sub_ =
      nh_.subscribe<nav_msgs::Odometry>("/blue2/ground_truth/state", 1, &Blackboard::Blue2GroundTruthCallback, this);

  {
    my_color_ = RED;
    p_my_robot1_->SetRobotType(RED_1);
    p_my_robot1_->SetRobotType(RED_2);
    enemy_robot_1_.SetRobotType(BLUE_1);
    enemy_robot_2_.SetRobotType(BLUE_2);
  }

  vec_buff_zone_status_.resize(7);
}

Blackboard::~Blackboard() = default;

const std::shared_ptr<MyRobot> &Blackboard::GetMyRobot1() {
  return p_my_robot1_;
}

const std::shared_ptr<MyRobot> &Blackboard::GetMyRobot2() {
  return p_my_robot2_;
}

const EnemyRobot &Blackboard::GetEnemyRobot1() const {
  return enemy_robot_1_;
}

const EnemyRobot &Blackboard::GetEnemyRobot2() const {
  return enemy_robot_2_;
}

const std::vector<BuffZoneStatus> &Blackboard::GetBuffZoneStatus() const {
  return vec_buff_zone_status_;
}

MyColor Blackboard::GetMyColor() const {
  return my_color_;
}

void Blackboard::OutpostDetectedCallback(const roborts_msgs::OutpostDetected::ConstPtr &msg) {

  if (my_color_ == UNKNOWN_COLOR) {
    return;
  }

  if (my_color_ == RED) {
    enemy_robot_1_.SetIsDetected(msg->blue1_detected);
    enemy_robot_1_.SetPose(msg->header.stamp, msg->blue1);
    enemy_robot_2_.SetIsDetected(msg->blue2_detected);
    enemy_robot_2_.SetPose(msg->header.stamp, msg->blue2);
  } else if (my_color_ == BLUE) {
    enemy_robot_1_.SetIsDetected(msg->red1_detected);
    enemy_robot_1_.SetPose(msg->header.stamp, msg->red1);
    enemy_robot_2_.SetIsDetected(msg->red2_detected);
    enemy_robot_2_.SetPose(msg->header.stamp, msg->red2);
  }
}

void Blackboard::BuffZoneStatusCallback(const roborts_msgs::BuffZoneStatus::ConstPtr &msg) {
  vec_buff_zone_status_.at(1).is_active_ = msg->F1_zone_status;
  vec_buff_zone_status_.at(2).is_active_ = msg->F2_zone_status;
  vec_buff_zone_status_.at(3).is_active_ = msg->F3_zone_status;
  vec_buff_zone_status_.at(4).is_active_ = msg->F4_zone_status;
  vec_buff_zone_status_.at(5).is_active_ = msg->F5_zone_status;
  vec_buff_zone_status_.at(6).is_active_ = msg->F6_zone_status;

  vec_buff_zone_status_.at(1).buff_status_ = static_cast<BuffStatus>(msg->F1_zone_buff_debuff_status);
  vec_buff_zone_status_.at(2).buff_status_ = static_cast<BuffStatus>(msg->F2_zone_buff_debuff_status);
  vec_buff_zone_status_.at(3).buff_status_ = static_cast<BuffStatus>(msg->F3_zone_buff_debuff_status);
  vec_buff_zone_status_.at(4).buff_status_ = static_cast<BuffStatus>(msg->F4_zone_buff_debuff_status);
  vec_buff_zone_status_.at(5).buff_status_ = static_cast<BuffStatus>(msg->F5_zone_buff_debuff_status);
  vec_buff_zone_status_.at(6).buff_status_ = static_cast<BuffStatus>(msg->F6_zone_buff_debuff_status);
}

void Blackboard::MyColorCallback(const roborts_msgs::RobotStatus::ConstPtr &msg) {
  // if (msg->id == RED_1 || msg->id == RED_2) {
  my_color_ = RED;
  p_my_robot1_->SetRobotType(RED_1);
  p_my_robot1_->SetRobotType(RED_2);
  enemy_robot_1_.SetRobotType(BLUE_1);
  enemy_robot_2_.SetRobotType(BLUE_2);
  // } else if (msg->id == BLUE_1 || msg->id == BLUE_2) {
  //   my_color_ = BLUE;
  //   p_my_robot1_->SetRobotType(BLUE_1);
  //   p_my_robot1_->SetRobotType(BLUE_2);
  //   enemy_robot_1_.SetRobotType(RED_1);
  //   enemy_robot_2_.SetRobotType(RED_2);
  // } else {
  //   my_color_ = UNKNOWN_COLOR;
  // }
}

void Blackboard::GameSurvivorCallback(const roborts_msgs::GameSurvivor::ConstPtr &msg) {
  if (my_color_ == RED) {
    // TODO 自动区分1车和2车
    p_my_robot1_->SetIsSurvival(msg->red3);
    p_my_robot1_->SetIsSurvival(msg->red4);
    enemy_robot_1_.SetIsSurvival(msg->blue3);
    enemy_robot_2_.SetIsSurvival(msg->blue4);
  } else if (my_color_ == BLUE) {
    p_my_robot1_->SetIsSurvival(msg->blue3);
    p_my_robot1_->SetIsSurvival(msg->blue4);
    enemy_robot_1_.SetIsSurvival(msg->red3);
    enemy_robot_2_.SetIsSurvival(msg->red4);
  }
}

void Blackboard::Blue1GroundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  enemy_robot_1_.SetPose(msg->header.stamp, msg->pose.pose.position);
}

void Blackboard::Blue2GroundTruthCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  enemy_robot_2_.SetPose(msg->header.stamp, msg->pose.pose.position);
}





