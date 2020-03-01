//
// Created by kehan on 2020/2/28.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "blackboard_common.h"

namespace roborts_decision {
class EnemyRobot {
 public:
  explicit EnemyRobot(const RobotId &robot_id);
  virtual ~EnemyRobot();

  RobotId GetId() const;

  RobotType GetRobotType() const;
  void SetRobotType(RobotType robot_type);

  bool IsSurvival() const;
  void SetIsSurvival(bool is_survival);

  const geometry_msgs::PoseStamped &GetPose() const;
  void SetPose(const ros::Time &stamp, const geometry_msgs::Point &position);

  bool IsDetected() const;
  void SetIsDetected(bool is_detected);

 private:
  RobotId id_;
  RobotType robot_type_;

  bool is_survival_;
  bool is_detected_;
  geometry_msgs::PoseStamped pose_;
};
}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_
