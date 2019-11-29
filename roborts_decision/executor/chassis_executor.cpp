#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include "io/io.h"
#include "chassis_executor.h"
#include "../proto/decision.pb.h"
#include "../proto/controller.pb.h"
#include "../interface/roborts_dynamic_reconfigure.h"

namespace roborts_decision {

ChassisExecutor::ChassisExecutor() : execution_mode_(ExcutionMode::IDLE_MODE), execution_state_(BehaviorState::IDLE),
                                     global_planner_client_("global_planner_node_action", true),
                                     local_planner_client_("local_planner_node_action", true) {
  ros::NodeHandle nh;
  cmd_vel_acc_pub_ = nh.advertise<roborts_msgs::TwistAccel>("cmd_vel_acc", 100);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("odom", 100, &ChassisExecutor::ChassisOdomCallback, this);
  global_planner_client_.waitForServer();
  ROS_INFO("Global planer server start!");
  local_planner_client_.waitForServer();
  ROS_INFO("Local planer server start!");

  if (!LoadParam(ros::package::getPath("roborts_decision") + "/config/chassis_executor.prototxt")) {
    ROS_ERROR("%s can't open file", __FUNCTION__);
  }
}

bool ChassisExecutor::LoadParam(const std::string &proto_file_path) {
  roborts_decision::ControllerConfig controller_config;
  if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &controller_config)) {
    return false;
  }

  this->chassis_v2p_pid_kp = controller_config.pid_controller().chassis_p();
  this->chassis_v2p_pid_ki = controller_config.pid_controller().chassis_i();
  this->chassis_v2p_pid_kd = controller_config.pid_controller().chassis_d();
  this->chassis_v2p_pid_has_threshold = controller_config.pid_controller().chassis_has_threshold();
  this->chassis_v2p_pid_threshold = controller_config.pid_controller().chassis_threshold();

  return true;
}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal) {
  execution_mode_ = ExcutionMode::GOAL_MODE;
  global_planner_goal_.goal = goal;
  global_planner_client_.sendGoal(global_planner_goal_,
                                  GlobalActionClient::SimpleDoneCallback(),
                                  GlobalActionClient::SimpleActiveCallback(),
                                  boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
}

void ChassisExecutor::Execute(const geometry_msgs::PoseStamped &goal, GoalMode _goal_mode) {
  printf("Now in the Execute1");
  if (_goal_mode == GoalMode::GOAL_MODE_USE_GOLBAL_LOCAL_PLANNER) {
    execution_mode_ = ExcutionMode::GOAL_MODE;
    global_planner_goal_.goal = goal;
    global_planner_client_.sendGoal(global_planner_goal_,
                                    GlobalActionClient::SimpleDoneCallback(),
                                    GlobalActionClient::SimpleActiveCallback(),
                                    boost::bind(&ChassisExecutor::GlobalPlannerFeedbackCallback, this, _1));
  } else if (_goal_mode == GoalMode::GOAL_MODE_USE_ODOM_DATA) {

    if (execution_mode_ == ExcutionMode::GOAL_MODE) {
      Cancel();
    }

    printf("Now in the GOAL_FROM_ODOM_MODE \n");
    execution_mode_ = ExcutionMode::GOAL_FROM_ODOM_MODE;

    //TODO
    this->chassis_v2p_pid_kp = roborts_decision::roborts_dynamic_reconfigure::getInstance()->GetChassisV2PPidKp();
    this->chassis_v2p_pid_ki = roborts_decision::roborts_dynamic_reconfigure::getInstance()->GetChassisV2PPidKi();
    this->chassis_v2p_pid_kd = roborts_decision::roborts_dynamic_reconfigure::getInstance()->GetChassisV2PPidKd();
    this->chassis_v2p_pid_has_threshold =
        roborts_decision::roborts_dynamic_reconfigure::getInstance()->IsChassisV2PHasThreshold();
    this->chassis_v2p_pid_threshold =
        roborts_decision::roborts_dynamic_reconfigure::getInstance()->GetChassisV2PThreshold();

    static roborts_common::firefly::PIDController pid_controller_toward_angular(chassis_v2p_pid_kp,
                                                                                chassis_v2p_pid_ki,
                                                                                chassis_v2p_pid_kd,
                                                                                chassis_v2p_pid_has_threshold,
                                                                                chassis_v2p_pid_threshold);

    pid_controller_toward_angular.kp = chassis_v2p_pid_kp;
    pid_controller_toward_angular.ki = chassis_v2p_pid_ki;
    pid_controller_toward_angular.kd = chassis_v2p_pid_kd;
    pid_controller_toward_angular.has_threshold = chassis_v2p_pid_has_threshold;
    pid_controller_toward_angular.threshold = chassis_v2p_pid_threshold;

    geometry_msgs::Twist vel;

    auto chassis_yaw = tf::getYaw(this->chassis_odom_.pose.pose.orientation);
    auto goal_yaw = tf::getYaw(goal.pose.orientation);
    chassis_yaw = pid_controller_toward_angular.convertCurYaw2FabsYawThetaBetweenPI(goal_yaw, chassis_yaw);

    printf("chassis_yaw = %lf \n", chassis_yaw);
    
    pid_controller_toward_angular.setTarget(tf::getYaw(goal.pose.orientation));
    printf("goal_yaw = %lf \n", tf::getYaw(goal.pose.orientation));
    pid_controller_toward_angular.update(chassis_yaw);

    vel.linear.x = 0.0;
    vel.linear.y = 0.0;
    vel.angular.x = 0.0;
    vel.angular.y = 0.0;
    vel.angular.z = pid_controller_toward_angular.output();
    printf("output vel = %lf \n", vel.angular.z);
    cmd_vel_pub_.publish(vel);

  }
}
void ChassisExecutor::Execute(const geometry_msgs::Twist &twist) {
  if (execution_mode_ == ExcutionMode::GOAL_MODE) {
    Cancel();
  }
  execution_mode_ = ExcutionMode::SPEED_MODE;
  cmd_vel_pub_.publish(twist);
}

void ChassisExecutor::Execute(const roborts_msgs::TwistAccel &twist_accel) {
  if (execution_mode_ == ExcutionMode::GOAL_MODE) {
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

    case ExcutionMode::GOAL_MODE:state = global_planner_client_.getState();
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

    case ExcutionMode::SPEED_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::SPEED_WITH_ACCEL_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    case ExcutionMode::GOAL_FROM_ODOM_MODE:execution_state_ = BehaviorState::RUNNING;
      break;

    default:ROS_ERROR("Wrong Execution Mode");
  }
  return execution_state_;

};

void ChassisExecutor::Cancel() {
  switch (execution_mode_) {
    case ExcutionMode::IDLE_MODE:ROS_WARN("Nothing to be canceled.");
      break;

    case ExcutionMode::GOAL_MODE:global_planner_client_.cancelGoal();
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

    case ExcutionMode::GOAL_FROM_ODOM_MODE:cmd_vel_acc_pub_.publish(zero_twist_accel_);
      execution_mode_ = ExcutionMode::IDLE_MODE;
      usleep(50000);
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

void ChassisExecutor::ChassisOdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  this->chassis_odom_ = *msg;
}
}
