//
// Created by kehan on 2020/4/12.
//

#include "robot_behaviors.h"

using namespace roborts_decision;

roborts_decision::RobotBehaviors::RobotBehaviors(std::shared_ptr<MyRobot> p_my_robot,
                                                 std::shared_ptr<Blackboard> p_blackboard) {

}

const std::shared_ptr<GoalBehavior> &roborts_decision::RobotBehaviors::GetGoalBehavior() {
  return p_goal_behavior_;
}

const std::shared_ptr<EscapeBehavior> &roborts_decision::RobotBehaviors::GetEscapeBehavior() {
  return p_escape_behavior_;
}

const std::shared_ptr<SupplyBehavior> &roborts_decision::RobotBehaviors::GetSupplyBehavior() {
  return p_supply_behavior_;
}

const std::shared_ptr<PursueAttackBehavior> &roborts_decision::RobotBehaviors::GetPursueAttackBehavior() {
  return p_pursue_attack_behavior_;
}
