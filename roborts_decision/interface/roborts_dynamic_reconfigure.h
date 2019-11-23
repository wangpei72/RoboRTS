//
// Created by heihei on 2019/11/23.
//

#ifndef SRC_ROBORTS_ROBORTS_DECISION_INTERFACE_ROBORTS_DYNAMIC_RECONFIGURE_H_
#define SRC_ROBORTS_ROBORTS_DECISION_INTERFACE_ROBORTS_DYNAMIC_RECONFIGURE_H_

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "roborts_decision/roborts_dynamic_cfgConfig.h"

namespace roborts_decision {
class roborts_dynamic_reconfigure {

 public:

  static roborts_dynamic_reconfigure *getInstance();

  virtual ~roborts_dynamic_reconfigure();

  int8_t update();

  double GetChassisV2PPidKp() const;
  double GetChassisV2PPidKi() const;
  double GetChassisV2PPidKd() const;
  bool IsChassisV2PHasThreshold() const;
  double GetChassisV2PThreshold() const;

 private:

  roborts_dynamic_reconfigure();

  roborts_dynamic_reconfigure(const roborts_dynamic_reconfigure &);

  roborts_dynamic_reconfigure &operator=(const roborts_dynamic_reconfigure &);

  static roborts_dynamic_reconfigure *instance;
  static boost::mutex mutex_instance;

  void reconfig_cb(roborts::roborts_dynamic_cfgConfig &_config, uint32_t _level);

  ros::NodeHandle nh;
  dynamic_reconfigure::Server<roborts::roborts_dynamic_cfgConfig> dyconfig_server;
  dynamic_reconfigure::Server<roborts::roborts_dynamic_cfgConfig>::CallbackType dyconfig_cb_type;

  double chassis_v2p_pid_kp;
  double chassis_v2p_pid_ki;
  double chassis_v2p_pid_kd;

  bool chassis_v2p_has_threshold;
  double chassis_v2p_threshold;

};

}


#endif //SRC_ROBORTS_ROBORTS_DECISION_INTERFACE_ROBORTS_DYNAMIC_RECONFIGURE_H_
