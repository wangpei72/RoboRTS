//
// Created by kehan on 19-7-11.
//

#ifndef ROBORTS_COMMON_PIDCONTROLLER_H_
#define ROBORTS_COMMON_PIDCONTROLLER_H_

#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace roborts_common {
namespace firefly {

class PIDController {
 public:

  PIDController(double_t _kp, double_t _ki, double_t _kd, bool _has_threshold = false, double_t _threshold = 0)
      : has_threshold(_has_threshold),
        threshold(_threshold),
        kp(_kp),
        ki(_ki),
        kd(_kd),
        target(0.),
        err(0.),
        ierr(0.),
        derr(0.),
        last_err(0.) {

  }

  virtual ~PIDController() = default;

  int8_t setTarget(double_t _target) {
    target = _target;
    ierr = 0;
    return 0;
  }

  double_t getTarget() {
    return this->target;
  }

  int8_t update(double_t _cur) {
    err = target - _cur;
    ierr += err;
    derr = err - last_err;
    last_err = err;
    return 0;
  }

  double_t output() {
    double_t ans = (kp * err + ki * ierr + kd * derr);
    if (has_threshold && (fabs(ans) > fabs(threshold))) {
      ans = (ans / fabs(ans)) * threshold;          // TODO Maybe not 1?
    }
    return ans;
  }



  bool has_threshold;
  double_t threshold;

  double_t kp;
  double_t ki;
  double_t kd;

 private:

  double_t target;

  double_t err;
  double_t ierr;
  double_t derr;

  double_t last_err;

};

}
}

#endif //ROBORTS_COMMON_PIDCONTROLLER_H_
