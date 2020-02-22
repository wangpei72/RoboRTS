//
// Created by heihei on 2019/11/23.
//

#ifndef SRC_ROBORTS_ROBORTS_DECISION_INTERFACE_ROBORTS_DYNAMIC_RECONFIGURE_H_
#define SRC_ROBORTS_ROBORTS_DECISION_INTERFACE_ROBORTS_DYNAMIC_RECONFIGURE_H_

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>

#include "roborts_decision/roborts_dynamic_cfgConfig.h"

namespace roborts_common {
namespace firefly {
class DynamicReconfigureInterface {

 public:

  static DynamicReconfigureInterface *getInstance();

  virtual ~DynamicReconfigureInterface();

  int8_t update();

  double GetV2PPidKp() const;
  double GetV2PPidKi() const;
  double GetV2PPidKd() const;
  bool IsV2PHasThreshold() const;
  double GetV2PThreshold() const;
  double GetYawGaolTolerance() const;
  const std::string &GetServerName() const;
  const std::string &GetPublisherName() const;
  const std::string &GetSubscriberName() const;
 private:

  DynamicReconfigureInterface();

  DynamicReconfigureInterface(const DynamicReconfigureInterface &);

  DynamicReconfigureInterface &operator=(const DynamicReconfigureInterface &);

  static DynamicReconfigureInterface *instance;
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
  double yaw_gaol_tolerance;

  std::string server_name;
  std::string publisher_name;
  std::string subscriber_name;

};
}
}


#endif //SRC_ROBORTS_ROBORTS_DECISION_INTERFACE_ROBORTS_DYNAMIC_RECONFIGURE_H_
