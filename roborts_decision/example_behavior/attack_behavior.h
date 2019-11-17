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

    auto chassis_cur_map_yaw = tf::getYaw(blackboard_->GetChassisMapPose().pose.orientation);
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

    std::cout << terminal_io_color::RED << "residual_yaw" << residual_yaw << terminal_io_color::BLANK << std::endl;

    roborts_msgs::GimbalAngle residual_gimbal_angle{};
    residual_gimbal_angle.yaw_mode = 0;
    residual_gimbal_angle.pitch_mode = 0;
    residual_gimbal_angle.pitch_angle = gimbal_goal_map_pitch;
    residual_gimbal_angle.yaw_angle = kscale * residual_yaw;

    gimbal_executor_->Execute(residual_gimbal_angle);
  }

  void ChassisRotationAction() {

    auto chassis_executor_state = ChassisExecutorUpdate();
    if (chassis_executor_state != BehaviorState::RUNNING) {
      static int point_index = 0;
      auto new_goal = chassis_rot_points.at(point_index);
      chassis_executor_->Execute(new_goal);
      point_index = (++point_index) % 2;
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

    auto tmp_goal_orientation = tf::createQuaternionMsgFromYaw(1.);
    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
    this->chassis_rot_points.emplace_back(tmp_goal_pose);

    tmp_goal_orientation = tf::createQuaternionMsgFromYaw(-1.);
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
};
}
}

#endif //ROBORTS_DECISION_ATTACK_BEHAVIOR_H_
