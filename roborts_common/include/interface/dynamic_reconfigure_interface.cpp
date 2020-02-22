//
// Created by heihei on 2019/11/23.
//

#include "dynamic_reconfigure_interface.h"

roborts_common::firefly::DynamicReconfigureInterface::DynamicReconfigureInterface() {

  nh = ros::NodeHandle("~");

  // this ptr.
  dyconfig_cb_type = boost::bind(&roborts_common::firefly::DynamicReconfigureInterface::reconfig_cb, this, _1, _2);
  dyconfig_server.setCallback(dyconfig_cb_type);
}

roborts_common::firefly::DynamicReconfigureInterface::DynamicReconfigureInterface(const roborts_common::firefly::DynamicReconfigureInterface &) {

}

roborts_common::firefly::DynamicReconfigureInterface &roborts_common::firefly::DynamicReconfigureInterface::operator=(
    const roborts_common::firefly::DynamicReconfigureInterface &) {

}

roborts_common::firefly::DynamicReconfigureInterface::~DynamicReconfigureInterface() {
  delete instance;
}

roborts_common::firefly::DynamicReconfigureInterface
    *roborts_common::firefly::DynamicReconfigureInterface::instance = nullptr;

boost::mutex roborts_common::firefly::DynamicReconfigureInterface::mutex_instance;

roborts_common::firefly::DynamicReconfigureInterface *roborts_common::firefly::DynamicReconfigureInterface::getInstance() {
  if (instance == nullptr) {
    boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
    if (instance == nullptr) {
      instance = new DynamicReconfigureInterface();
    }
  }

  return instance;
}

int8_t roborts_common::firefly::DynamicReconfigureInterface::update() {
  try {
    ros::spinOnce();
  }
  catch (ros::Exception &ex) {
    ROS_ERROR("DynamicRecfgInterface Update Error: %s", ex.what());
    return -1;
  }

  return 0;
}

void roborts_common::firefly::DynamicReconfigureInterface::reconfig_cb(roborts::roborts_dynamic_cfgConfig &_config,
                                                                       uint32_t _level) {
  this->chassis_v2p_pid_kp = _config.chassis_p;
  this->chassis_v2p_pid_ki = _config.chassis_i;
  this->chassis_v2p_pid_kd = _config.chassis_d;

  this->chassis_v2p_has_threshold = _config.chassis_has_threshold;
  this->chassis_v2p_threshold = _config.chassis_threshold;

  this->server_name = _config.server_name;
  this->publisher_name = _config.publisher_name;
  this->subscriber_name = _config.subscriber_name;

  this->yaw_gaol_tolerance = _config.yaw_goal_tolerance;

}

double roborts_common::firefly::DynamicReconfigureInterface::GetV2PPidKp() const {
  return chassis_v2p_pid_kp;
}
double roborts_common::firefly::DynamicReconfigureInterface::GetV2PPidKi() const {
  return chassis_v2p_pid_ki;
}
double roborts_common::firefly::DynamicReconfigureInterface::GetV2PPidKd() const {
  return chassis_v2p_pid_kd;
}
bool roborts_common::firefly::DynamicReconfigureInterface::IsV2PHasThreshold() const {
  return chassis_v2p_has_threshold;
}
double roborts_common::firefly::DynamicReconfigureInterface::GetV2PThreshold() const {
  return chassis_v2p_threshold;
}
double roborts_common::firefly::DynamicReconfigureInterface::GetYawGaolTolerance() const {
  return yaw_gaol_tolerance;
}
const std::string &roborts_common::firefly::DynamicReconfigureInterface::GetServerName() const {
  return server_name;
}
const std::string &roborts_common::firefly::DynamicReconfigureInterface::GetPublisherName() const {
  return publisher_name;
}
const std::string &roborts_common::firefly::DynamicReconfigureInterface::GetSubscriberName() const {
  return subscriber_name;
}
