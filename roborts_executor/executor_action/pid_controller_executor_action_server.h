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

  PIDControllerExecuteActionServer(ros::NodeHandle nh,
                                   std::string action_name,
                                   const std::string& output_name,
                                   const std::string& input_name) :
      action_server_(nh,
                     std::move(action_name),
                     boost::bind(&PIDControllerExecuteActionServer::pid_controller_execute, this, _1), false) {

    action_server_.registerPreemptCallback(boost::bind(&PIDControllerExecuteActionServer::preemptCallBack, this));

    cmd_vel_pub_ =
        nh.advertise<geometry_msgs::Twist>(output_name, 1);
    pose_sub_ =
        nh.subscribe<geometry_msgs::PoseStamped>(input_name, 1, &PIDControllerExecuteActionServer::ClientPoseCallback, this);

    action_server_.start();
  }

  ~PIDControllerExecuteActionServer() = default;

  void pid_controller_execute(const roborts_msgs::PIDControllerTowardAngularGoalConstPtr &pid_controller_toward_angular_goal) {
//    ROS_INFO("Received goal!");
    roborts_msgs::PIDControllerTowardAngularFeedback feedback;
    ros::Rate rate(50);

    auto now_yaw = tf::getYaw(this->client_pose_.pose.orientation);
    auto goal_yaw = tf::getYaw(pid_controller_toward_angular_goal->goal.pose.orientation);
    auto chassis_yaw = roborts_common::firefly::convertCurYaw2FabsYawThetaBetweenPI(goal_yaw, now_yaw);

    static roborts_common::firefly::PIDController pid_controller_toward_angular
        (roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PPidKp(),
         roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PPidKi(),
         roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PPidKd(),
         roborts_common::firefly::DynamicReconfigureInterface::getInstance()->IsChassisV2PHasThreshold(),
         roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PThreshold());

    printf("chassis_yaw - goal_yaw = %lf", chassis_yaw - goal_yaw);
    double difference_yaw = chassis_yaw - goal_yaw;

    while (fabs(chassis_yaw - goal_yaw) >  0.8){
//    while (difference_yaw * (chassis_yaw - goal_yaw) > 0) {

      pid_controller_toward_angular.SetKp(roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PPidKp());
      pid_controller_toward_angular.SetKi(roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PPidKi());
      pid_controller_toward_angular.SetKd(roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PPidKd());
      pid_controller_toward_angular.SetHasThreshold(roborts_common::firefly::DynamicReconfigureInterface::getInstance()->IsChassisV2PHasThreshold());
      pid_controller_toward_angular.SetThreshold(roborts_common::firefly::DynamicReconfigureInterface::getInstance()->GetChassisV2PThreshold());

      geometry_msgs::Twist vel;

      chassis_yaw = tf::getYaw(this->client_pose_.pose.orientation);
      goal_yaw = tf::getYaw(pid_controller_toward_angular_goal->goal.pose.orientation);
      chassis_yaw = roborts_common::firefly::convertCurYaw2FabsYawThetaBetweenPI(goal_yaw, chassis_yaw);

      if (!action_server_.isActive()) {
        break;
      }

      pid_controller_toward_angular.setTarget(tf::getYaw(pid_controller_toward_angular_goal->goal.pose.orientation));
      printf("goal_yaw = %lf \n", tf::getYaw(pid_controller_toward_angular_goal->goal.pose.orientation));
      pid_controller_toward_angular.update(chassis_yaw);

      vel.linear.x = 0.0;
      vel.linear.y = 0.0;
      vel.angular.x = 0.0;
      vel.angular.y = 0.0;
      vel.angular.z = pid_controller_toward_angular.output();
      printf("output vel = %lf \n", vel.angular.z);
      cmd_vel_pub_.publish(vel);
      difference_yaw = chassis_yaw - goal_yaw;

      printf("chassis_yaw = %lf \n", chassis_yaw);
      feedback.differ_angle = chassis_yaw;
      action_server_.publishFeedback(feedback);

      rate.sleep();

    }

    ROS_INFO("Complete!");
    action_server_.setSucceeded();

  }

  void preemptCallBack() {
    ROS_INFO("Preempted");
    if (action_server_.isActive()) {
      action_server_.setPreempted();
    }
  }

  void ClientPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    printf("received the pose !!!!!!");
    this->client_pose_ = *msg;
  }

 private:

  actionlib::SimpleActionServer<roborts_msgs::PIDControllerTowardAngularAction> action_server_;

  ros::Subscriber pose_sub_;

  geometry_msgs::PoseStamped client_pose_;
  //  nav_msgs::Odometry client_pose_;

  ros::Publisher cmd_vel_pub_;

};

}
}
#endif //SRC_ROBORTS_ROBORTS_EXECUTOR_EXECUTOR_ACTION_PID_CONTROLLER_EXECUTOR_ACTION_SERVER_H_
