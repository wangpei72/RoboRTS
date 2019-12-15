//
// Created by heihei on 2019/12/14.
//

#ifndef SRC_ROBORTS_ROBORTS_EXECUTOR_EXECUTOR_ACTION_PID_CONTROLLER_EXECUTOR_ACTION_SERVER_H_
#define SRC_ROBORTS_ROBORTS_EXECUTOR_EXECUTOR_ACTION_PID_CONTROLLER_EXECUTOR_ACTION_SERVER_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "utils/utils.h"
#include "roborts_msgs/PIDControllerTowardAngularAction.h"
#include "pid_controller/pid_controller.h"
#include "interface/dynamic_reconfigure_interface.h"

namespace roborts_executor {
namespace firefly {

class PIDControllerExecuteActionServer {

 public:

  PIDControllerExecuteActionServer(ros::NodeHandle nh, std::string action_name);

  virtual ~PIDControllerExecuteActionServer();

  void pid_controller_execute(const roborts_msgs::PIDControllerTowardAngularGoalConstPtr &pid_controller_toward_angular_goal);

  void preemptCallBack();

  void ClientPoseCallback(const nav_msgs::Odometry::ConstPtr &msg);

 private:

  actionlib::SimpleActionServer<roborts_msgs::PIDControllerTowardAngularAction> action_server_;

  ros::Subscriber pose_sub_;

  nav_msgs::Odometry client_pose_;

  ros::Publisher cmd_vel_pub_;

};

}
}
#endif //SRC_ROBORTS_ROBORTS_EXECUTOR_EXECUTOR_ACTION_PID_CONTROLLER_EXECUTOR_ACTION_SERVER_H_
