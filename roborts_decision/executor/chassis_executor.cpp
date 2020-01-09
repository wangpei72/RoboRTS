#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include "io/io.h"
#include "chassis_executor.h"
#include "../proto/decision.pb.h"

namespace roborts_decision {

ChassisExecutor::ChassisExecutor() : execution_mode_(ExcutionMode::IDLE_MODE), execution_state_(BehaviorState::IDLE),
                                     global_planner_client_("global_planner_node_action", true),
                                     local_planner_client_("local_planner_node_action", true),
                                     pid_controller_client_("pid_planner_chassis_node_action", true) {
  ros::NodeHandle nh;
  cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  global_planner_client_.waitForServer();
  ROS_INFO("Global planer server start!");
  local_planner_client_.waitForServer();
  ROS_INFO("Local planer server start!");
  pid_controller_client_.waitForServer();
  ROS_INFO("PID controller chassis server start!");

//  if (!LoadParam(ros::package::getPath("roborts_decision") + "/config/chassis_executor.prototxt")) {
//    ROS_ERROR("%s can't open file", __FUNCTION__);
//  }
}

//bool ChassisExecutor::LoadParam(const std::string &proto_file_path) {
//  roborts_decision::ControllerConfig controller_config;
//  if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &controller_config)) {
//    return false;
//  }

//  this->chassis_v2p_pid_kp = controller_config.pid_controller().chassis_p();
//  this->chassis_v2p_pid_ki = controller_config.pid_controller().chassis_i();
//  this->chassis_v2p_pid_kd = controller_config.pid_controller().chassis_d();
//  this->chassis_v2p_pid_has_threshold = controller_config.pid_controller().chassis_has_threshold();
//  this->chassis_v2p_pid_threshold = controller_config.pid_controller().chassis_threshold();
//
//  return true;
//}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal) {

  if (execution_mode_ == ExcutionMode::GOAL_FROM_ODOM_MODE) {
    Cancel();
  }

  execution_mode_ = ExcutionMode::GOAL_USE_PLANNER_MODE;
  global_planner_goal_.goal = goal;
  global_planner_client_.sendGoal(global_planner_goal_,
                                  GlobalActionClient::SimpleDoneCallback(),
                                  GlobalActionClient::SimpleActiveCallback(),
                                  boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal, GoalMode _goal_mode) {
  if (_goal_mode == GoalMode::GOAL_MODE_USE_GOLBAL_LOCAL_PLANNER) {

    if (execution_mode_ == ExcutionMode::GOAL_FROM_ODOM_MODE) {
      Cancel();
    }
    execution_mode_ = ExcutionMode::GOAL_USE_PLANNER_MODE;
    global_planner_goal_.goal = goal;
    global_planner_client_.sendGoal(global_planner_goal_,
                                    GlobalActionClient::SimpleDoneCallback(),
                                    GlobalActionClient::SimpleActiveCallback(),
                                    boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
  } else if (_goal_mode == GoalMode::GOAL_MODE_USE_ODOM_DATA) {

    if (execution_mode_ == ExcutionMode::GOAL_USE_PLANNER_MODE) {
      Cancel();
    }
    execution_mode_ = ExcutionMode::GOAL_FROM_ODOM_MODE;

    pid_controller_toward_angular_goal_.goal = goal;

    static int number = 0;
    if (number % 250 == 0 or pid_controller_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {

      ROS_ERROR("Send the chassis goal");
      pid_controller_client_.sendGoal(pid_controller_toward_angular_goal_,
                                      PIDControllerClient::SimpleDoneCallback(),
                                      PIDControllerClient::SimpleActiveCallback(),
                                      boost::bind(&ChassisExecutor::PIDControllerFeedbackCallback, this, _1));
      number = 0;
    }
    number = number + 1;

  }
}

void ChassisExecutor::Execute(const geometry_msgs::Twist &twist) {
  if (execution_mode_ == ExcutionMode::GOAL_USE_PLANNER_MODE) {
    Cancel();
  }
  if (execution_mode_ == ExcutionMode::GOAL_FROM_ODOM_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  cmd_vel_pub_.publish(twist);
}

void ChassisExecutor::Execute(const roborts_msgs::TwistAccel &twist_accel) {
  if (execution_mode_ == ExcutionMode::GOAL_USE_PLANNER_MODE) {
    Cancel();
  }
  if (execution_mode_ == ExcutionMode::GOAL_FROM_ODOM_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_WITH_ACCEL_MODE;

  cmd_vel_acc_pub_.publish(twist_accel);
}

BehaviorState ChassisExecutor::Update() {
  actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;
  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:execution_state_ = BehaviorState::IDLE;
      break;

    case ExcutionMode::GOAL_USE_PLANNER_MODE:state = global_planner_client_.getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_INFO("%s : ACTIVE", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("%s : PENDING", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("%s : SUCCEEDED", __FUNCTION__);
        execution_state_ = BehaviorState::SUCCESS;

      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("%s : ABORTED", __FUNCTION__);
        execution_state_ = BehaviorState::FAILURE;

      } else {
        ROS_ERROR("Error: %s", state.toString().c_str());
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::GOAL_FROM_ODOM_MODE:state = pid_controller_client_.getState();
      if (state == actionlib::SimpleClientGoalState::ACTIVE) {
        ROS_INFO("%s : pid_controller_chassis_client ACTIVE", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;
      } else if (state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("%s : pid_controller_chassis_client PENDING", __FUNCTION__);
        execution_state_ = BehaviorState::RUNNING;

      } else if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("%s : pid_controller_chassis_client SUCCEEDED", __FUNCTION__);
        execution_state_ = BehaviorState::SUCCESS;

      } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
        ROS_INFO("%s : pid_controller_chassis_client ABORTED", __FUNCTION__);
        execution_state_ = BehaviorState::FAILURE;

      } else {
        ROS_ERROR("pid_controller_chassis_client Error: %s", state.toString().c_str());
        execution_state_ = BehaviorState::FAILURE;
      }
      break;

    case ExcutionMode::SPEED_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;

};

void ChassisExecutor::Cancel() {
  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::GOAL_USE_PLANNER_MODE:global_planner_client_.cancelGoal();
      local_planner_client_.cancelGoal();
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_MODE:cmd_vel_pub_.publish(zero_twist_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:cmd_vel_acc_pub_.publish(zero_twist_accel_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      usleep(50000);
      break;

    case ExcutionMode::GOAL_FROM_ODOM_MODE:pid_controller_client_.cancelGoal();
      execution_mode_ = ExcutionMode::IDLE_MODE;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }

}

void ChassisExecutor::GlobalPlannerFeedbackCallback(const roborts_msgs::GlobalPlannerFeedbackConstPtr &global_planner_feedback) {
  if (!global_planner_feedback->path.poses.empty()) {
    local_planner_goal_.route = global_planner_feedback->path;
    local_planner_client_.sendGoal(local_planner_goal_);
  }
}

void ChassisExecutor::PIDControllerFeedbackCallback(const roborts_msgs::PIDControllerTowardAngularFeedbackConstPtr &pid_controller_toward_angular_feedback) {
//  printf("The differ angle is %lf \n", pid_controller_toward_angular_feedback->differ_angle);
}

}
