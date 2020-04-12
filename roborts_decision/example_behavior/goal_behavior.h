#ifndef ROBORTS_DECISION_GOAL_BEHAVIOR_H
#define ROBORTS_DECISION_GOAL_BEHAVIOR_H

#include <utility>

#include "io/io.h"

#include "../blackboard/blackboard_raw.h"
#include "../executor/chassis_executor.h"
#include "../behavior_tree/behavior_state.h"

namespace roborts_decision {
class GoalBehavior {
 public:
  explicit GoalBehavior(std::shared_ptr<ChassisExecutor> p_chassis_executor) :
      p_chassis_executor_(std::move(p_chassis_executor)) {}

  void Run(const geometry_msgs::PoseStamped &goal) {
    p_chassis_executor_->Execute(goal);
  }

  void Cancel() {
    p_chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return p_chassis_executor_->Update();
  }

  ~GoalBehavior() = default;

 private:
  //! executor
  std::shared_ptr<ChassisExecutor>  p_chassis_executor_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;

};
}

#endif //ROBORTS_DECISION_GOAL_BEHAVIOR_H
