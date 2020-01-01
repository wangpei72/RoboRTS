#include "gimbal_executor.h"
namespace roborts_decision {
GimbalExecutor::GimbalExecutor() : excution_mode_(ExcutionMode::IDLE_MODE),
                                   execution_state_(BehaviorState::IDLE),
                                   pid_controller_client_("pid_planner_gimbal_node_action", true) {
  ros::NodeHandle nh;
  cmd_gimbal_angle_pub_ = nh.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 1);
  cmd_gimbal_rate_pub_ = nh.advertise<roborts_msgs::GimbalRate>("cmd_gimbal_rate", 1);

  pid_controller_client_.waitForServer();
  ROS_INFO("PID controller gimbal server start!");

}

void GimbalExecutor::Execute(const roborts_msgs::GimbalAngle &gimbal_angle) {

  if (excution_mode_ == ExcutionMode::PID_MODE) {
    Cancel();
  }

  excution_mode_ = ExcutionMode::ANGLE_MODE;
  cmd_gimbal_angle_pub_.publish(gimbal_angle);
}

void GimbalExecutor::Execute(const roborts_msgs::GimbalRate &gimbal_rate) {

  if (excution_mode_ == ExcutionMode::PID_MODE) {
    Cancel();
  }

  excution_mode_ = ExcutionMode::RATE_MODE;
  cmd_gimbal_rate_pub_.publish(gimbal_rate);
}

void GimbalExecutor::Execute(const geometry_msgs::PoseStamped &gimbal_angle, GoalMode _goal_mode) {

  printf("Now in the Execute add \n");
  if (_goal_mode == GoalMode::GOAL_MODE_USE_PID) {

    printf("Now in the Gimbal control mode --USING PID \n");
    excution_mode_ = ExcutionMode::PID_MODE;

    pid_controller_toward_angular_goal_.goal = gimbal_angle;
    pid_controller_client_.sendGoal(pid_controller_toward_angular_goal_,
                                    PIDControllerClient::SimpleDoneCallback(),
                                    PIDControllerClient::SimpleActiveCallback(),
                                    boost::bind(&GimbalExecutor::PIDControllerFeedbackCallback, this, _1));
  }
}

BehaviorState GimbalExecutor::Update() {
  switch (excution_mode_) {
    case ExcutionMode::IDLE_MODE:execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::ANGLE_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::RATE_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;
}

void GimbalExecutor::Cancel() {
  switch (excution_mode_) {
    case ExcutionMode::IDLE_MODE:ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::ANGLE_MODE:cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::RATE_MODE:cmd_gimbal_rate_pub_.publish(zero_gimbal_rate_);
      excution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }

}
void GimbalExecutor::PIDControllerFeedbackCallback(const roborts_msgs::PIDControllerTowardAngularFeedbackConstPtr &pid_controller_toward_angular_feedback) {
  printf("The differ angle is %lf", pid_controller_toward_angular_feedback->differ_angle);
  //TODO
}

}