#include "vm_pid_linear.h"

VmPidLinear::VmPidLinear(){
  gain_p = 10.0;
  gain_d = 5.0;
  gain_i = 0.01;
  sp_range = 0.002;
}

double VmPidLinear::update(double set_point, double cur_state, double dt){
  cur_state = f_round(cur_state, 3);
  
  // update proportional, differential and integral errors
  p = set_point - cur_state;	// current error
  if(fabs(p) <= sp_range) p = 0.0;
  i = i + dt * p;				// i -> sum of prev errors
  d = (p - prevErr) / dt;		// d -> rate of error
   
  prevErr = p;
  
  // update control output
  output = gain_p * p + gain_d * d + gain_i * i;
  
  return output;
}

void VmPidLinear::reset(){
  p = i = d = output = prevErr = 0;
}

double VmPidLinear::f_round(double f, int decimals){	
    float value = (int)(f * pow(10,decimals) + .5); 
    return (float)value / pow(10,decimals); 
}

