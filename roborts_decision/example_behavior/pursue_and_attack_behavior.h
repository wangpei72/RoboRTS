//
// Created by heihei on 2020/4/12.
//

#ifndef ICRA_FIREFLY_ROBORTS_PURSUE_AND_ATTACK_BEHAVIOR_H_
#define ICRA_FIREFLY_ROBORTS_PURSUE_AND_ATTACK_BEHAVIOR_H_

#include <utility>

#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"

namespace roborts_decision {

class PursueAndAttackBehavior {
 public:
  PursueAndAttackBehavior(std::shared_ptr<ChassisExecutor> p_chassis_executor,
                          std::shared_ptr<Blackboard> p_blackboard,
                          std::shared_ptr<MyRobot> p_my_robot)
      : p_chassis_executor_(std::move(p_chassis_executor)),
        p_blackboard_(std::move(p_blackboard)),
        p_my_robot_(std::move(p_my_robot)) {
    ros::NodeHandle nh;
  }

  void Run(RobotId enemy_id, double aim_distance) {
    if (getDistance(enemy_id) > aim_distance) {
      geometry_msgs::PoseStamped goal_pose;
      getGoalPose(enemy_id, aim_distance, goal_pose);
      this->p_my_robot_->GetPChassisExecutor()->Execute(goal_pose);
    } else {
      //Attack the enemy robot
    }
  }

  void Cancel() {
    this->p_my_robot_->GetPChassisExecutor()->Cancel();
  }

  BehaviorState Update() {
  }

  ~PursueAndAttackBehavior() = default;

 private:
  double getDistance(RobotId enemy_id) {
    double my_robot_x = 0, my_robot_y = 0, enemy_robot_x = 0, enemy_robot_y = 0;

    my_robot_x = this->p_my_robot_->GetChassisMapPose().pose.position.x;
    my_robot_y = this->p_my_robot_->GetChassisMapPose().pose.position.y;

    if (enemy_id == ENEMY_ROBOT_1) {
      enemy_robot_x = this->p_blackboard_->GetEnemyRobot1().GetPose().pose.position.x;
      enemy_robot_y = this->p_blackboard_->GetEnemyRobot1().GetPose().pose.position.y;
    } else if (enemy_id == ENEMY_ROBOT_2) {
      enemy_robot_x = this->p_blackboard_->GetEnemyRobot2().GetPose().pose.position.x;
      enemy_robot_y = this->p_blackboard_->GetEnemyRobot2().GetPose().pose.position.y;
    } else {
      ROS_ERROR("PursueAndAttack get the error enemy_robot_id !");
    }

    return sqrt((enemy_robot_x - my_robot_x) * (enemy_robot_x - my_robot_x)
                    + (enemy_robot_y - my_robot_y) * (enemy_robot_y - my_robot_y));
  }

  void getGoalPose(RobotId enemy_id, double aim_distance, geometry_msgs::PoseStamped &pose_stamped) {
    double my_robot_x = 0, my_robot_y = 0, enemy_robot_x = 0, enemy_robot_y = 0;

    my_robot_x = this->p_my_robot_->GetChassisMapPose().pose.position.x;
    my_robot_y = this->p_my_robot_->GetChassisMapPose().pose.position.y;

    if (enemy_id == ENEMY_ROBOT_1) {
      enemy_robot_x = this->p_blackboard_->GetEnemyRobot1().GetPose().pose.position.x;
      enemy_robot_y = this->p_blackboard_->GetEnemyRobot1().GetPose().pose.position.y;
    } else if (enemy_id == ENEMY_ROBOT_2) {
      enemy_robot_x = this->p_blackboard_->GetEnemyRobot2().GetPose().pose.position.x;
      enemy_robot_y = this->p_blackboard_->GetEnemyRobot2().GetPose().pose.position.y;
    } else {
      ROS_ERROR("PursueAndAttack get the error enemy_robot_id !");
    }

    double distance = 0.0;
    distance = getDistance(enemy_id);

    pose_stamped.pose.position.x = my_robot_x + (distance - aim_distance) / distance * (enemy_robot_x - my_robot_x);
    pose_stamped.pose.position.y = my_robot_y + (distance - aim_distance) / distance * (enemy_robot_y - my_robot_y);
    pose_stamped.pose.orientation.w = 1;
  }

  std::shared_ptr<ChassisExecutor> p_chassis_executor_;
  std::shared_ptr<Blackboard> p_blackboard_;
  std::shared_ptr<MyRobot> p_my_robot_;

  std::vector<geometry_msgs::PoseStamped> chassis_rot_points{};
};
}

#endif //ICRA_FIREFLY_ROBORTS_PURSUE_AND_ATTACK_BEHAVIOR_H_
