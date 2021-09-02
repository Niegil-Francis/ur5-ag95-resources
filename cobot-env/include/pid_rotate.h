#ifndef PID_ROTATE_H
#define PID_ROTATE_H

#include <cmath>
#include <iostream>

/**
 * @brief PID controller class for rotational motion of human_drone.
 */

class VmPidRotate {
protected:
	double f_round(double f, int decimals);

public:
  VmPidRotate();

  double gain_p;
  double gain_i;
  double gain_d;
  double sp_range; // set point +- range

  double prevErr;
  double output;
  double p, i, d;

  double update(double set_point, double cur_state, double dt);
  void reset();
};

#endif // VM_PID_ROTATE_H
