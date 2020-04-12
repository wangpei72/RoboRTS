//
// Created by kehan on 2020/4/12.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_ROBOT_BEHAVIORS_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_ROBOT_BEHAVIORS_H_

#include "my_robot.h"
#include "blackboard.h"
#include "../example_behavior/goal_behavior.h"
#include "../example_behavior/escape_behavior.h"
#include "../example_behavior/supply_behavior.h"
#include "../example_behavior/chase_behavior.h"
#include <thread>

namespace roborts_decision {

class RobotBehaviors {
 public:
  RobotBehaviors(std::shared_ptr<MyRobot> p_my_robot,
                 std::shared_ptr<Blackboard> p_blackboard);

  const std::shared_ptr<GoalBehavior> &GetGoalBehavior();
  const std::shared_ptr<EscapeBehavior> &GetEscapeBehavior();
  const std::shared_ptr<SupplyBehavior> &GetSupplyBehavior();
  const std::shared_ptr<PursueAttackBehavior> &GetPursueAttackBehavior();

 private:
  std::shared_ptr<MyRobot> p_my_robot_;
  std::shared_ptr<GoalBehavior> p_goal_behavior_;
  std::shared_ptr<EscapeBehavior> p_escape_behavior_;
  std::shared_ptr<SupplyBehavior> p_supply_behavior_;
  std::shared_ptr<PursueAttackBehavior> p_pursue_attack_behavior_;
};

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_ROBOT_BEHAVIORS_H_
