//
// Created by kehan on 2020/2/28.
//

#include "blackboard_raw.h"

roborts_decision::BlackboardRaw::BlackboardRaw(const std::string &proto_file_path) :
    enemy_detected_(false),
    armor_detection_actionlib_client_("armor_detection_node_action", true) {
  tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  ros::NodeHandle ns_nh;
  std::string config_dir;
  ns_nh.getParam("config_dir", config_dir);
  std::cout << "!!!! Get config_dir:" << config_dir << std::endl;
  std::string map_path = ros::package::getPath("roborts_costmap") + \
      "/config/" + config_dir + "/costmap_parameter_config_for_decision.prototxt";
  costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                           map_path);
  charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();

  costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();

  // Enemy fake pose
  ros::NodeHandle rviz_nh("/move_base_simple");
  enemy_sub_ = rviz_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, &BlackboardRaw::GoalCallback, this);

  ros::NodeHandle nh;

  roborts_decision::DecisionConfig decision_config;
  roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config);

  if (!decision_config.simulate()) {

    armor_detection_actionlib_client_.waitForServer();

    ROS_INFO("Armor detection module has been connected!");

    armor_detection_goal_.command = 1;
    armor_detection_actionlib_client_.sendGoal(armor_detection_goal_,
                                               actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleDoneCallback(),
                                               actionlib::SimpleActionClient<roborts_msgs::ArmorDetectionAction>::SimpleActiveCallback(),
                                               boost::bind(&BlackboardRaw::ArmorDetectionFeedbackCallback, this, _1));
  }

}

roborts_decision::BlackboardRaw::~BlackboardRaw() {

}

void roborts_decision::BlackboardRaw::ArmorDetectionFeedbackCallback(const roborts_msgs::ArmorDetectionFeedbackConstPtr &feedback) {
  if (feedback->detected) {
    enemy_detected_ = true;
    ROS_INFO("Find Enemy!");

    tf::Stamped<tf::Pose> tf_pose, global_tf_pose;
    geometry_msgs::PoseStamped camera_pose_msg, global_pose_msg;
    camera_pose_msg = feedback->enemy_pos;

    double distance = std::sqrt(camera_pose_msg.pose.position.x * camera_pose_msg.pose.position.x +
        camera_pose_msg.pose.position.y * camera_pose_msg.pose.position.y);
    double yaw = atan(camera_pose_msg.pose.position.y / camera_pose_msg.pose.position.x);

    //camera_pose_msg.pose.position.z=camera_pose_msg.pose.position.z;
    tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,
                                                            0,
                                                            yaw);
    camera_pose_msg.pose.orientation.w = quaternion.w();
    camera_pose_msg.pose.orientation.x = quaternion.x();
    camera_pose_msg.pose.orientation.y = quaternion.y();
    camera_pose_msg.pose.orientation.z = quaternion.z();
    poseStampedMsgToTF(camera_pose_msg, tf_pose);

    tf_pose.stamp_ = ros::Time(0);
    try {
      tf_ptr_->transformPose("map", tf_pose, global_tf_pose);
      tf::poseStampedTFToMsg(global_tf_pose, global_pose_msg);

      if (GetDistance(global_pose_msg, enemy_pose_) > 0.2 || GetAngle(global_pose_msg, enemy_pose_) > 0.2) {
        enemy_pose_ = global_pose_msg;

      }
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("tf error when transform enemy pose from camera to map");
    }
  } else {
    enemy_detected_ = false;
  }

}

geometry_msgs::PoseStamped roborts_decision::BlackboardRaw::GetEnemy() const {
  return enemy_pose_;
}

bool roborts_decision::BlackboardRaw::IsEnemyDetected() const {
  ROS_INFO("%s: %d", __FUNCTION__, (int) enemy_detected_);
  return enemy_detected_;
}

void roborts_decision::BlackboardRaw::GoalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal) {
  new_goal_ = true;
  goal_ = *goal;
}

geometry_msgs::PoseStamped roborts_decision::BlackboardRaw::GetGoal() const {
  return goal_;
}

bool roborts_decision::BlackboardRaw::IsNewGoal() {
  if (new_goal_) {
    new_goal_ = false;
    return true;
  } else {
    return false;
  }
}

double roborts_decision::BlackboardRaw::GetDistance(const geometry_msgs::PoseStamped &pose1,
                                                    const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Point point1 = pose1.pose.position;
  const geometry_msgs::Point point2 = pose2.pose.position;
  const double dx = point1.x - point2.x;
  const double dy = point1.y - point2.y;
  return std::sqrt(dx * dx + dy * dy);
}

double roborts_decision::BlackboardRaw::GetAngle(const geometry_msgs::PoseStamped &pose1,
                                                 const geometry_msgs::PoseStamped &pose2) {
  const geometry_msgs::Quaternion quaternion1 = pose1.pose.orientation;
  const geometry_msgs::Quaternion quaternion2 = pose2.pose.orientation;
  tf::Quaternion rot1, rot2;
  tf::quaternionMsgToTF(quaternion1, rot1);
  tf::quaternionMsgToTF(quaternion2, rot2);
  return rot1.angleShortestPath(rot2);
}

const geometry_msgs::PoseStamped roborts_decision::BlackboardRaw::GetChassisMapPose() {
  UpdateChassisPose();
  return chassis_map_pose_;
}

const geometry_msgs::PoseStamped roborts_decision::BlackboardRaw::GetGimbalMapPose() {
  UpdateGimbalPose();
  return gimbal_map_pose_;
}

const std::shared_ptr<roborts_decision::BlackboardRaw::CostMap> roborts_decision::BlackboardRaw::GetCostMap() {
  return costmap_ptr_;
}

const roborts_decision::BlackboardRaw::CostMap2D* roborts_decision::BlackboardRaw::GetCostMap2D() {
  return costmap_2d_;
}

const unsigned char* roborts_decision::BlackboardRaw::GetCharMap() {
  return charmap_;
}

void roborts_decision::BlackboardRaw::UpdateChassisPose() {
  tf::Stamped<tf::Pose> chassis_tf_pose;
  chassis_tf_pose.setIdentity();

  chassis_tf_pose.frame_id_ = "base_link";
  chassis_tf_pose.stamp_ = ros::Time();
  try {
    geometry_msgs::PoseStamped chassis_pose;
    tf::poseStampedTFToMsg(chassis_tf_pose, chassis_pose);
    tf_ptr_->transformPose("map", chassis_pose, chassis_map_pose_);
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
  }
}

void roborts_decision::BlackboardRaw::UpdateGimbalPose() {
  tf::Stamped<tf::Pose> gimbal_tf_pose;
  gimbal_tf_pose.setIdentity();

  gimbal_tf_pose.frame_id_ = "gimbal";
  gimbal_tf_pose.stamp_ = ros::Time();
  try {
    geometry_msgs::PoseStamped gimbal_pose;
    tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
    tf_ptr_->transformPose("map", gimbal_pose, gimbal_map_pose_);
  }
  catch (tf::LookupException &ex) {
    ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
  }
}
