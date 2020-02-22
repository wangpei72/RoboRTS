//
// Created by heihei on 2019/12/15.
//
#include <ros/ros.h>
#include "executor_action/pid_controller_executor_action_server.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "gimbal_executor_action_node");
  ros::NodeHandle nh;

  roborts_executor::firefly::PIDControllerExecuteActionServer
      action_server(nh, "pid_planner_gimbal_node_action", "/cmd_gimbal_speed", "/gimbal_pose");

  ros::spin();
  return 0;
}
