//
// Created by kehan on 2019/11/3.
//

#ifndef ROBORTS_DECISION_ATTACK_BEHAVIOR_H_
#define ROBORTS_DECISION_ATTACK_BEHAVIOR_H_

#include "io/io.h"

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

  void Run() {

  }

  void SetGimbalMapPose(const double &gimbal_goal_map_pitch, const double &gimbal_goal_map_yaw) {

    // TODO
    auto gimbal_executor_state = GimbalExecutorUpdate();

    auto chassis_cur_map_yaw = tf::getYaw(blackboard_->GetRobotMapPose().pose.orientation);

    auto chassis_cur_map_yaw_pose = tf::createQuaternionFromYaw(chassis_cur_map_yaw);
    auto gimbal_goal_map_yaw_pose = tf::createQuaternionFromYaw(gimbal_goal_map_yaw);
    auto residual_yaw = gimbal_goal_map_yaw_pose.angleShortestPath(chassis_cur_map_yaw_pose);

    roborts_msgs::GimbalAngle residual_gimbal_angle;
    residual_gimbal_angle.pitch_angle = gimbal_goal_map_pitch;
    residual_gimbal_angle.yaw_angle = residual_yaw;

    gimbal_executor_->Execute(residual_gimbal_angle);
  }

  void ChassisRotationAction() {

    auto chassis_executor_state = ChassisExecutorUpdate();

    if (chassis_executor_state != BehaviorState::RUNNING) {

    }
  }

  void resetRotationPoint(geometry_msgs::Point new_point) {

    this->chassis_rot_points.clear();

    geometry_msgs::PoseStamped tmp_goal_pose;
    tmp_goal_pose.header.frame_id = "map";
    tmp_goal_pose.pose.position = new_point;

//    geometry_msgs::Quaternion tmp_goal_attitude;
    auto tmp_goal_orientation = tf::createQuaternionMsgFromYaw(0.);
    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
    this->chassis_rot_points.emplace_back(tmp_goal_pose);

    tmp_goal_orientation = tf::createQuaternionMsgFromYaw(1.);
    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
    this->chassis_rot_points.emplace_back(tmp_goal_pose);

    tmp_goal_orientation = tf::createQuaternionMsgFromYaw(-1.);
    tmp_goal_pose.pose.orientation = tmp_goal_orientation;
    this->chassis_rot_points.emplace_back(tmp_goal_pose);
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
