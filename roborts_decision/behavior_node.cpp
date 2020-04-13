//
// Created by kehan on 2020/4/12.
//

#include <ros/ros.h>

#include "example_behavior/escape_behavior.h"
#include "example_behavior/pursue_attack_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/supply_behavior.h"

#include "blackboard/robot_behaviors.h"
#include "blackboard/my_robot.h"
#include "blackboard/blackboard.h"

using namespace roborts_decision;

int main(int argc, char** argv) {

  ros::init(argc, argv, "behavior_test_node");
  auto p_my_robot1 = std::make_shared<MyRobot>(MY_ROBOT_1, ros::NodeHandle("/red1"));
  auto p_my_robot2 = std::make_shared<MyRobot>(MY_ROBOT_2, ros::NodeHandle("/red2"));

  auto p_blackboard = std::make_shared<Blackboard>(p_my_robot1, p_my_robot2);
  auto p_robot1_behaviors = std::make_shared<RobotBehaviors>(p_my_robot1, p_blackboard);
  auto p_robot2_behaviors = std::make_shared<RobotBehaviors>(p_my_robot2, p_blackboard);

  std::cout << "Start send goal..." << std::endl;
  geometry_msgs::PoseStamped goal;
  goal.pose.position.x = -3.4;
  goal.pose.position.y = -1.7;
  goal.pose.position.z = 0;
  goal.pose.orientation.w = 1;
  p_robot1_behaviors->GetGoalBehavior()->Run(goal);

  /*
   * Example:
   * 在赋予机器人一个逃跑动作的时候，先判断是否有当前动作:
   * auto current_behavior = p_my_robot1->GetCurrentBehavior();
   * 发现此时机器人的动作为SUPPLY，然后先取消当前动作：
   * p_robot1_behaviors->GetSupplyBehavior()->Cancel();
   * 然后设定当前的逃跑动作：
   * p_my_robot1->SetCurrentBehavior(ESCAPE);
   * p_robot1_behaviors->GetEscapeBehavior()->Run();
   */

  /*
   * PS：如果选取对应机器人不方便的话，可以加上map等数据结构，这个比如map_behaviors{RobotID::MY_ROBOT_1, p_robot1_behaviors}
   *   RobotId hp_high_robot = MY_ROBOT_1;
   *   RobotId hp_low_robot = MY_ROBOT_2;
   *   map_behaviors.at(hp_low_robot)->GetEscapeBehavior()->Run();
   */
  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std::cout << "behavior status: " << p_robot1_behaviors->GetStatusCode() << std::endl;
    loop_rate.sleep();
  }
}




