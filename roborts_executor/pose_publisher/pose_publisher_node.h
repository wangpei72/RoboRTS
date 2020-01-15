//
// Created by heihei on 2019/12/29.
//

#ifndef CHASSIS_POSE_PUBLISHER_CHASSIS_POSE_PUBLISHER_NODE_H_
#define CHASSIS_POSE_PUBLISHER_CHASSIS_POSE_PUBLISHER_NODE_H_

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"

class PosePublisher {
 public:
  PosePublisher() {

    tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
    chassis_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/chassis_pose", 1);
    gimbal_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/gimbal_pose", 1);

  }

  ~PosePublisher() = default;

  void Update() {
    UpdateChassisPose();
    UpdateGimbalPose();
    chassis_pose_pub_.publish(this->chassis_map_pose_);
    gimbal_pose_pub_.publish(this->gimbal_map_pose_);
  }

 private:

  void UpdateChassisPose() {
    tf::Stamped<tf::Pose> chassis_tf_pose;
    chassis_tf_pose.setIdentity();

    chassis_tf_pose.frame_id_ = "base_link";
    chassis_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped chassis_pose;
      tf::poseStampedTFToMsg(chassis_tf_pose, chassis_pose);
      tf_ptr_->transformPose("map", chassis_pose, chassis_map_pose_);
      ROS_WARN("chassis pose %lf", tf::getYaw(chassis_map_pose_.pose.orientation));
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
    }
  }

  void UpdateGimbalPose() {
    tf::Stamped<tf::Pose> gimbal_tf_pose;
    gimbal_tf_pose.setIdentity();

    gimbal_tf_pose.frame_id_ = "gimbal";
    gimbal_tf_pose.stamp_ = ros::Time();
    try {
      geometry_msgs::PoseStamped gimbal_pose;
      tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
      tf_ptr_->transformPose("map", gimbal_pose, gimbal_map_pose_);
      ROS_WARN("gimbal pose %lf", tf::getYaw(gimbal_map_pose_.pose.orientation));
    }
    catch (tf::LookupException &ex) {
      ROS_ERROR("Transform Error looking up gimbal pose: %s", ex.what());
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("Transform Error looking up chassis pose: %s", ex.what());
    }
  }
  //! tf
  std::shared_ptr<tf::TransformListener> tf_ptr_;

  ros::NodeHandle nh;
  ros::Publisher chassis_pose_pub_;
  ros::Publisher gimbal_pose_pub_;

  geometry_msgs::PoseStamped chassis_map_pose_;
  geometry_msgs::PoseStamped gimbal_map_pose_;

};

#endif //CHASSIS_POSE_PUBLISHER_CHASSIS_POSE_PUBLISHER_NODE_H_
