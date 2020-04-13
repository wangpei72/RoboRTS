//
// Created by kehan on 2020/4/12.
//

#ifndef ROBORTS_ROBORTS_DECISION_EXAMPLE_BEHAVIOR_SUPPLY_BEHAVIOR_H_
#define ROBORTS_ROBORTS_DECISION_EXAMPLE_BEHAVIOR_SUPPLY_BEHAVIOR_H_

namespace roborts_decision {

class SupplyBehavior {
 public:
  SupplyBehavior(ChassisExecutor*&chassis_executor,
                 BlackboardRaw*&blackboard) :
      chassis_executor_(chassis_executor),
      blackboard_(blackboard) {}

  void Run() {
    if (blackboard_->IsNewGoal()) {
      chassis_executor_->Execute(blackboard_->GetGoal());
    }
  }

  void Cancel() {
    chassis_executor_->Cancel();
  }

  BehaviorState Update() {
    return chassis_executor_->Update();
  }

  ~SupplyBehavior() = default;
 private:
  //! executor
  ChassisExecutor* const chassis_executor_;

  //! perception information
  BlackboardRaw* const blackboard_;

  //! planning goal
  geometry_msgs::PoseStamped planning_goal_;
};

}

#endif //ROBORTS_ROBORTS_DECISION_EXAMPLE_BEHAVIOR_SUPPLY_BEHAVIOR_H_
