//
// Created by kehan on 2020/2/28.
//

#include "my_robot.h"
using namespace roborts_decision;

MyRobot::MyRobot(RobotId id, const ros::NodeHandle &nh) :
    nh_(nh),
    id_(id),
    remaining_hp_(2000),
    max_hp_(2000),
    current_heat_(0),
    is_survival_(true),
    remaining_projectiles_(0),
    no_move_(false),
    no_shoot_(false) {

  robot_type_ = RobotType::UNKNOWN_TYPE;
//  if (id_ == RED1 || id_ == BLUE1) {
//    remaining_projectiles_ = 50;
//  } else {
//    remaining_projectiles_ = 0;
//  }

  p_chassis_executor_ = std::make_shared<ChassisExecutor>(nh_);
  // p_gimbal_executor_ = std::make_shared<GimbalExecutor>(nh_);

  // TODO
  current_behavior_ = MyRobotBehavior::GOAL;

  std::string armors_under_attack_topic("armors_under_attack");
  nh_.param("armors_under_attack_topic", armors_under_attack_topic, armors_under_attack_topic);
  armors_under_attack_sub_ =
      nh_.subscribe<roborts_msgs::RobotDamage>(armors_under_attack_topic, 1, &MyRobot::ArmorsUnderAttackCallback, this);

  std::string heat_topic("heat");
  nh_.param("heat_topic", heat_topic, heat_topic);
  heat_sub_ =
      nh_.subscribe<roborts_msgs::RobotHeat>(heat_topic, 1, &MyRobot::HeatCallback, this);

  std::string armors_in_eyes_topic("armors_in_eyes");
  nh_.param("armors_in_eyes_topic", armors_in_eyes_topic, armors_in_eyes_topic);
  armors_in_eyes_sub_ =
      nh_.subscribe<roborts_msgs::ArmorsDetected>(armors_in_eyes_topic, 1, &MyRobot::ArmorsInEyesCallback, this);

  std::string robot_status_topic("robot_status");
  nh_.param("robot_status_topic", robot_status_topic, robot_status_topic);
  robot_status_sub_ =
      nh_.subscribe<roborts_msgs::RobotStatus>(robot_status_topic, 1, &MyRobot::RobotStatusCallback, this);

  robot_map_pose_sub_ =
      nh_.subscribe<geometry_msgs::PoseStamped>("amcl_pose", 1, &MyRobot::ChassisMapPoseCallback, this);

  // tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
  // tf_thread_ptr_ = std::make_shared<std::thread>([&]() {
  //   ros::Rate loop_rate(30);
  //   while (ros::ok()) {
  //     UpdateChassisMapPose();
  //     UpdateChassisOdomPose();
  //     UpdateGimbalMapPose();
  //     UpdateGimbalOdomPose();
  //     loop_rate.sleep();
  //   }
  // });
  // tf_thread_ptr_->detach();
  p_ros_spin_thread_ = std::make_shared<std::thread>([&]() {
    ros::Rate loop_rate(10);
    while (ros::ok()) {
      ros::spinOnce();
      loop_rate.sleep();
    }
  });
  p_ros_spin_thread_->detach();
}

MyRobot::~MyRobot() = default;

void MyRobot::ArmorsUnderAttackCallback(const roborts_msgs::RobotDamage::ConstPtr &msg) {
  // TODO
}

void MyRobot::HeatCallback(const roborts_msgs::RobotHeat::ConstPtr &msg) {
  current_heat_ = msg->shooter_heat;
}

void MyRobot::RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr &msg) {
  robot_type_ = static_cast<RobotType>(msg->id);
  remaining_hp_ = msg->remain_hp;
  max_hp_ = msg->max_hp;
  no_move_ = !msg->chassis_output;
  no_shoot_ = !msg->shooter_output;
}

void MyRobot::ArmorsInEyesCallback(const roborts_msgs::ArmorsDetected::ConstPtr &msg) {
  armors_in_eyes_ = *msg;
}

RobotId MyRobot::GetId() const {
  return id_;
}

RobotType MyRobot::GetRobotType() const {
  return robot_type_;
}

void MyRobot::SetRobotType(RobotType robot_type) {
  robot_type_ = robot_type;
}

int MyRobot::GetHp() const {
  return remaining_hp_;
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

// const geometry_msgs::PoseStamped &MyRobot::GetChassisOdomPose() const {
//   return chassis_odom_pose_;
// }
//
// const geometry_msgs::PoseStamped &MyRobot::GetGimbalMapPose() const {
//   return gimbal_map_pose_;
// }
//
// const geometry_msgs::PoseStamped &MyRobot::GetGimbalOdomPose() const {
//   return gimbal_odom_pose_;
// }

// const geometry_msgs::PoseStamped &MyRobot::GetCurrentGoal() const {
//   return current_goal_;
// }

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

// ChassisExecutor* MyRobot::GetChassisExecutor() {
//   return &chassis_executor_;
// }
//
// GimbalExecutor* MyRobot::GetGimbalExecutor() {
//   return &gimbal_executor_;
// }

// void MyRobot::UpdateChassisMapPose() {
//   tf::Stamped<tf::Pose> chassis_tf_pose;
//   chassis_tf_pose.setIdentity();
//
//   chassis_tf_pose.frame_id_ = "base_link";
//   chassis_tf_pose.stamp_ = ros::Time();
//   try {
//     geometry_msgs::PoseStamped chassis_pose;
//     tf::poseStampedTFToMsg(chassis_tf_pose, chassis_pose);
//     tf_ptr_->transformPose("map", chassis_pose, chassis_map_pose_);
//   }
//   catch (tf::LookupException &ex) {
//     ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
//   }
// }

void MyRobot::ChassisMapPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  chassis_map_pose_ = *msg;
}

bool MyRobot::IsNoMove() const {
  return no_move_;
}

bool MyRobot::IsNoShoot() const {
  return no_shoot_;
}

std::shared_ptr<ChassisExecutor> MyRobot::GetPChassisExecutor() {
  return p_chassis_executor_;
}

std::shared_ptr<GimbalExecutor> MyRobot::GetPGimbalExecutor() {
  return p_gimbal_executor_;
}

// void MyRobot::UpdateChassisOdomPose() {
//   tf::Stamped<tf::Pose> chassis_tf_pose;
//   chassis_tf_pose.setIdentity();
//
//   chassis_tf_pose.frame_id_ = "base_link";
//   chassis_tf_pose.stamp_ = ros::Time();
//   try {
//     geometry_msgs::PoseStamped chassis_pose;
//     tf::poseStampedTFToMsg(chassis_tf_pose, chassis_pose);
//     tf_ptr_->transformPose("odom", chassis_pose, chassis_odom_pose_);
//   }
//   catch (tf::LookupException &ex) {
//     ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
//   }
// }

// void MyRobot::UpdateGimbalMapPose() {
//   tf::Stamped<tf::Pose> gimbal_tf_pose;
//   gimbal_tf_pose.setIdentity();
//
//   gimbal_tf_pose.frame_id_ = "gimbal";
//   gimbal_tf_pose.stamp_ = ros::Time();
//   try {
//     geometry_msgs::PoseStamped gimbal_pose;
//     tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
//     tf_ptr_->transformPose("map", gimbal_pose, gimbal_map_pose_);
//   }
//   catch (tf::LookupException &ex) {
//     ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
//   }
// }

// void MyRobot::UpdateGimbalOdomPose() {
//   tf::Stamped<tf::Pose> gimbal_tf_pose;
//   gimbal_tf_pose.setIdentity();
//
//   gimbal_tf_pose.frame_id_ = "gimbal";
//   gimbal_tf_pose.stamp_ = ros::Time();
//   try {
//     geometry_msgs::PoseStamped gimbal_pose;
//     tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
//     tf_ptr_->transformPose("odom", gimbal_pose, gimbal_odom_pose_);
//   }
//   catch (tf::LookupException &ex) {
//     ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
//   }
// }




