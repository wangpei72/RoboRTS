//
// Created by heihei on 2019/11/29.
//

#ifndef SRC_ROBORTS_ROBORTS_COMMON_INCLUDE_UTILS_UTILS_H_
#define SRC_ROBORTS_ROBORTS_COMMON_INCLUDE_UTILS_UTILS_H_

#include <cmath>
namespace roborts_common {
namespace firefly {

double_t convertCurYaw2FabsYawThetaBetweenPI(double_t _target_yaw, double_t _cur_yaw) {
  double_t yaw_theta = _target_yaw - _cur_yaw;

  while (yaw_theta > M_PI) {
    yaw_theta -= 2 * M_PI;
  }
  while (yaw_theta < -M_PI) {
    yaw_theta += 2 * M_PI;
  }

  return (_target_yaw - yaw_theta);
}
}
}
#endif //SRC_ROBORTS_ROBORTS_COMMON_INCLUDE_UTILS_UTILS_H_
