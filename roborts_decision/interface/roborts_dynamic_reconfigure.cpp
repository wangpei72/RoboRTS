//
// Created by heihei on 2019/11/23.
//

#include "roborts_dynamic_reconfigure.h"

roborts_decision::roborts_dynamic_reconfigure::roborts_dynamic_reconfigure() {

  nh = ros::NodeHandle("~");

  // this ptr.
  dyconfig_cb_type = boost::bind(&roborts_decision::roborts_dynamic_reconfigure::reconfig_cb, this, _1, _2);
  dyconfig_server.setCallback(dyconfig_cb_type);
}

roborts_decision::roborts_dynamic_reconfigure::roborts_dynamic_reconfigure(const roborts_decision::roborts_dynamic_reconfigure &) {

}

roborts_decision::roborts_dynamic_reconfigure &roborts_decision::roborts_dynamic_reconfigure::operator=(const roborts_decision::roborts_dynamic_reconfigure &) {

}

roborts_decision::roborts_dynamic_reconfigure::~roborts_dynamic_reconfigure() {
  delete instance;
}

roborts_decision::roborts_dynamic_reconfigure *roborts_decision::roborts_dynamic_reconfigure::instance = nullptr;

boost::mutex roborts_decision::roborts_dynamic_reconfigure::mutex_instance;

roborts_decision::roborts_dynamic_reconfigure *roborts_decision::roborts_dynamic_reconfigure::getInstance() {
  if (instance == nullptr) {
    boost::unique_lock<boost::mutex> uq_lock_instance(mutex_instance);
    if (instance == nullptr) {
      instance = new roborts_dynamic_reconfigure();
    }
  }

  return instance;
}

int8_t roborts_decision::roborts_dynamic_reconfigure::update() {
  try {
    ros::spinOnce();
  }
  catch (ros::Exception &ex) {
    ROS_ERROR("DynamicRecfgInterface Update Error: %s", ex.what());
    return -1;
  }

  return 0;
}

void roborts_decision::roborts_dynamic_reconfigure::reconfig_cb(roborts::roborts_dynamic_cfgConfig &_config,
                                                                uint32_t _level) {
  this->chassis_v2p_pid_kp = _config.chassis_p;
  this->chassis_v2p_pid_ki = _config.chassis_i;
  this->chassis_v2p_pid_kd = _config.chassis_d;

  this->chassis_v2p_has_threshold = _config.chassis_has_threshold;
  this->chassis_v2p_threshold = _config.chassis_threshold;

}

double roborts_decision::roborts_dynamic_reconfigure::GetChassisV2PPidKp() const {
  return chassis_v2p_pid_kp;
}
double roborts_decision::roborts_dynamic_reconfigure::GetChassisV2PPidKi() const {
  return chassis_v2p_pid_ki;
}
double roborts_decision::roborts_dynamic_reconfigure::GetChassisV2PPidKd() const {
  return chassis_v2p_pid_kd;
}
bool roborts_decision::roborts_dynamic_reconfigure::IsChassisV2PHasThreshold() const {
  return chassis_v2p_has_threshold;
}
double roborts_decision::roborts_dynamic_reconfigure::GetChassisV2PThreshold() const {
  return chassis_v2p_threshold;
}
