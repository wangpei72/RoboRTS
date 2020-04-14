//
// Created by kehan on 2020/2/29.
//

#ifndef ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_
#define ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_

#include <geometry_msgs/PoseStamped.h>
#include <roborts_msgs/ArmorsDetected.h>

namespace roborts_decision {

// TODO Add to roborts_common
enum MyRobotBehavior {
  CHASE,
  ESCAPE,
  GOAL,
  SUPPLY
};

enum ArmorId {
  FRONT = 0,
  RIGHT = 1,
  BACK = 2,
  LEFT = 3
};

enum BuffStatus {
  RED_RESTORATION = 1,
  RED_PROJECTILE_SUPPLIER = 2,
  BLUE_RESTORATION = 3,
  BLUE_PROJECTILE_SUPPLIER = 4,
  NO_SHOOTING = 5,
  NO_MOVING = 6,
  UNKNOWN_STATUS = 7
};

struct BuffZoneStatus {
  bool is_active_ = false;
  BuffStatus buff_status_ = UNKNOWN_STATUS;
};

enum RobotType {
  RED_1 = 3,
  RED_2 = 4,
  BLUE_1 = 13,
  BLUE_2 = 14,
  UNKNOWN_TYPE
};

enum RobotId {
  MY_ROBOT_1,
  MY_ROBOT_2,
  ENEMY_ROBOT_1,
  ENEMY_ROBOT_2,
};

enum MyColor {
  RED,
  BLUE,
  UNKNOWN_COLOR
};

enum Decision {
  Offensive
};

/*
bool IsPositionMsgEqual(const geometry_msgs::Point &position1,
                        const geometry_msgs::Point &position2) {
  return position1.x == position2.x &&
      position1.y == position2.y &&
      position1.z == position2.z;
}

bool IsQuaternionMsgEqual(const geometry_msgs::Quaternion &orientation1,
                          const geometry_msgs::Quaternion &orientation2) {
  return orientation1.x == orientation2.x &&
      orientation1.y == orientation2.y &&
      orientation1.z == orientation2.z &&
      orientation1.w == orientation2.w;
}

bool IsPoseMsgEqual(const geometry_msgs::PoseStamped &pose1,
                    const geometry_msgs::PoseStamped &pose2) {
  return IsPositionMsgEqual(pose1.pose.position, pose2.pose.position) &&
      IsQuaternionMsgEqual(pose1.pose.orientation, pose2.pose.orientation) &&
      pose1.header.frame_id == pose2.header.frame_id;
  // TODO Add stamp and seq to equal?
}

bool IsArmorMsgEqual(const roborts_msgs::ArmorDetected &armor1,
                     const roborts_msgs::ArmorDetected &armor2) {
  return armor1.robot_id == armor2.robot_id &&
      armor1.armor_id == armor2.armor_id &&
      IsPositionMsgEqual(armor1.position, armor2.position);
}

bool IsArmorsMsgEqual(const roborts_msgs::ArmorsDetected &armors1,
                      const roborts_msgs::ArmorsDetected &armors2) {

  if (armors1.armors.size() != armors2.armors.size()) {
    return false;
  }
  int index = 0;
  for (auto armor : armors1.armors) {
    if (!IsArmorMsgEqual(armor, armors2.armors.at(index))) {
      return false;
    }
    index++;
  }
  return true;
}
 */

}

#endif //ROBORTS_ROBORTS_DECISION_BLACKBOARD_BLACKBOARD_COMMON_H_