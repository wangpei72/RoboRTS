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

  bool IsSurvival() const;
  void SetIsSurvival(bool is_survival);

  const geometry_msgs::PoseStamped &GetPose() const;

 private:
  RobotId id_;
  bool is_survival_;
  geometry_msgs::PoseStamped pose_;
};
}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_ENEMY_ROBOT_H_
