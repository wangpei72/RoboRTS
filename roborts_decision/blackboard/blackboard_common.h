//
// Created by kehan on 2020/2/29.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_

namespace roborts_decision {

// TODO Add to roborts_common
enum RmRobotBehavior {
  ATTACK,
  BACK_BOOT_AREA,
  CHASE,
  ESCAPE,
  GOAL,
  PATROL,
  SEARCH
};

enum ArmorId {
  FRONT = 0,
  RIGHT = 1,
  BACK = 2,
  LEFT = 3
};

enum RobotId {
  RED1 = 3,
  RED2 = 4,
  BLUE1 = 13,
  BLUE2 = 14
};
}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_
