#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "executor/gimbal_executor.h"

#include "example_behavior/back_boot_area_behavior.h"
#include "example_behavior/escape_behavior.h"
#include "example_behavior/pursue_attack_behavior.h"
#include "example_behavior/search_behavior.h"
#include "example_behavior/patrol_behavior.h"
#include "example_behavior/goal_behavior.h"
#include "example_behavior/attack_behavior.h"

void Command();
char command = '0';
bool flag_restart_attack_behavior = false;

int _main(int argc, char **argv) {
  ros::init(argc, argv, "behavior_test_node");
  std::string full_path = ros::package::getPath("roborts_decision") + "/config/decision.prototxt";

  auto chassis_executor = new roborts_decision::ChassisExecutor;
  auto gimbal_executor = new roborts_decision::GimbalExecutor;
  auto blackboard = new roborts_decision::BlackboardRaw(full_path);

  roborts_decision::BackBootAreaBehavior back_boot_area_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PursueAttackBehavior chase_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::SearchBehavior search_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::EscapeBehavior escape_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::PatrolBehavior patrol_behavior(chassis_executor, blackboard, full_path);
  roborts_decision::GoalBehavior goal_behavior(chassis_executor, blackboard);
  roborts_decision::firefly::AttackBehavior attack_behavior(chassis_executor, gimbal_executor, blackboard);

  auto command_thread = std::thread(Command);
  ros::Rate rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    switch (command) {
      //back to boot area
      case '1':back_boot_area_behavior.Run();
        break;
        //patrol
      case '2':patrol_behavior.Run();
        break;
        //chase
      case '3':chase_behavior.Run();
        break;
        //search
      case '4':search_behavior.Run();
        break;
        //escape
      case '5':escape_behavior.Run();
        break;
        //goal
      case '6':goal_behavior.Run();
        break;
        // attack
      case '7':
        if (flag_restart_attack_behavior) {
          attack_behavior.Start();
          flag_restart_attack_behavior = false;
        }
        attack_behavior.Run(0., 0);
        break;
      case 27:
        if (command_thread.joinable()) {
          command_thread.join();
        }
        return 0;
      default:break;
    }
    rate.sleep();
  }

  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: back boot area behavior" << std::endl
              << "2: patrol behavior" << std::endl
              << "3: chase_behavior" << std::endl
              << "4: search behavior" << std::endl
              << "5: escape behavior" << std::endl
              << "6: goal behavior" << std::endl
              << "7: attack behavior" << std::endl
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    flag_restart_attack_behavior = (command == '7');

    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6'
        && command != '7' && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}

