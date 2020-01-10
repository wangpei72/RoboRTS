//
// Created by kehan on 2019/11/3.
//

#ifndef ROBORTS_DECISION_ATTACK_BEHAVIOR_H_
#define ROBORTS_DECISION_ATTACK_BEHAVIOR_H_

#include "io/io.h"
#include "io/terminal_io.h"
#include "pid_controller/pid_controller.h"

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "../behavior_tree/behavior_state.h"
#include "../proto/decision.pb.h"

#include "roborts_msgs/GimbalAngle.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

namespace roborts_decision {
namespace firefly {
class AttackBehavior {
 public:
  AttackBehavior(ChassisExecutor *&chassis_executor,
                 GimbalExecutor *&gimbal_executor,
                 Blackboard *blackboard) : chassis_executor_(chassis_executor),
                                           gimbal_executor_(gimbal_executor),
                                           blackboard_(blackboard) {
    ros::NodeHandle nh;
    chassis_pose_sub_ =
        nh.subscribe<geometry_msgs::PoseStamped>("/chassis_pose", 1,
                                                 &AttackBehavior::chassisPoseCallback, this);
  }

  void Start() {

    auto cur_chassis_map_pose = blackboard_->GetChassisMapPose();
    ResetRotationPoint(cur_chassis_map_pose.pose.position);
    std::cout << __FUNCTION__ << terminal_io_color::GREEN << "Start Attack Behavior" << terminal_io_color::BLANK
              << std::endl;
  }

  void Run(const double &gimbal_goal_map_pitch, const double &gimbal_goal_map_yaw) {
    ChassisRotationAction();
    SetGimbalMapPose(gimbal_goal_map_pitch, gimbal_goal_map_yaw);
  }

  void SetGimbalMapPose(const double &gimbal_goal_map_pitch, const double &gimbal_goal_map_yaw) {

    // TODO
    auto gimbal_executor_state = GimbalExecutorUpdate();

    // TODO change the chassis_pose source
    auto chassis_cur_map_yaw = tf::getYaw(chassis_pose_.pose.orientation);

    // TODO rename
    int kscale = 0;
    if (chassis_cur_map_yaw > 0) {
      kscale = -1;
    } else {
      kscale = 1;
    }

    auto chassis_cur_map_yaw_q = tf::createQuaternionFromYaw(chassis_cur_map_yaw);
    auto gimbal_goal_map_yaw_q = tf::createQuaternionFromYaw(gimbal_goal_map_yaw);
    auto residual_yaw = gimbal_goal_map_yaw_q.angleShortestPath(chassis_cur_map_yaw_q);

    ROS_WARN("chassis_cur_map_yaw = %lf  residual_yaw = %lf gimbal_goal_map_yaw = %lf",
             chassis_cur_map_yaw,
             kscale * residual_yaw,
             gimbal_goal_map_yaw);

//    roborts_msgs::GimbalAngle residual_gimbal_angle{};
//    residual_gimbal_angle.yaw_mode = 0;
//    residual_gimbal_angle.pitch_mode = 0;
//    residual_gimbal_angle.pitch_angle = gimbal_goal_map_pitch;
//    residual_gimbal_angle.yaw_angle = kscale * residual_yaw;

    geometry_msgs::PoseStamped residual_gimbal_angle;
    residual_gimbal_angle.pose.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(0, gimbal_goal_map_pitch, kscale * residual_yaw);
    gimbal_executor_->Execute(residual_gimbal_angle, GimbalExecutor::GoalMode::GOAL_MODE_USE_PID);
//    gimbal_executor_->Execute(residual_gimbal_angle);
  }

  void ChassisRotationAction() {

    auto chassis_executor_state = ChassisExecutorUpdate();
    static int point_index = 0;
    auto new_goal = chassis_rot_points.at(point_index);

    chassis_executor_->Execute(new_goal, ChassisExecutor::GoalMode::GOAL_MODE_USE_ODOM_DATA);
    if (chassis_executor_state == BehaviorState::SUCCESS) {

      point_index = (++point_index) % 2;
      printf("change the goal ----- in the ChassisRotationAction! \n");

    }

  }

  void ResetRotationPoint(geometry_msgs::Point new_point) {

    this->chassis_rot_points.clear();

    geometry_msgs::PoseStamped tmp_goal_pose;
    tmp_goal_pose.header.frame_id = "map";
    tmp_goal_pose.pose.position = new_point;

//    geometry_msgs::Quaternion tmp_goal_attitude{};
//    auto tmp_goal_orientation = tf::createQuaternionMsgFromYaw(0.);
//    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
//    this->chassis_rot_points.emplace_back(tmp_goal_pose);

    auto tmp_goal_orientation = tf::createQuaternionMsgFromYaw(1);
    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
    this->chassis_rot_points.emplace_back(tmp_goal_pose);

    tmp_goal_orientation = tf::createQuaternionMsgFromYaw(-1);
    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
    this->chassis_rot_points.emplace_back(tmp_goal_pose);
  }

  void Cancel() {
    chassis_executor_->Cancel();
    gimbal_executor_->Cancel();
  }

  BehaviorState ChassisExecutorUpdate() {
    return chassis_executor_->Update();
  }

  BehaviorState GimbalExecutorUpdate() {
    return gimbal_executor_->Update();
  }

  void chassisPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    this->chassis_pose_ = *msg;
  }

  ~AttackBehavior() = default;

 private:

  //! executor
  ChassisExecutor *const chassis_executor_;

  //! executor
  GimbalExecutor *const gimbal_executor_;

  //! perception information
  Blackboard *const blackboard_;

  //! chassis rotation points
  std::vector<geometry_msgs::PoseStamped> chassis_rot_points{};

  //! subscriber
  ros::Subscriber chassis_pose_sub_;

  //! chassis_pose
  geometry_msgs::PoseStamped chassis_pose_;

};
}
}

#endif //ROBORTS_DECISION_ATTACK_BEHAVIOR_H_
