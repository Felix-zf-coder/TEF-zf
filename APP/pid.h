#ifndef __PID_H__
#define __PID_H__
#include "stdint.h"
  typedef struct __PID_typedef{
	float Kp, Ki, Kd, max_out, max_Int, last_error, Int_error, time_const;
}PID_typedef;

  float PID_Compute(
	PID_typedef* pid, 
	float new_error);
  
#endif

