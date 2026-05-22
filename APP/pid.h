#ifndef __PID_H__
#define __PID_H__
#include "stdint.h"
  typedef struct __PID_typedef{
	float Kp, Ki, Kd, max_out, max_Int, last_error, Int_error, time_const;
}PID_typedef;

  float PID_Compute(
	PID_typedef* pid, 
	float new_error);



#define GIMBAL_ANGLE_PID_PARA \
{ \
    .Kp = 5.0f, \
    .Ki = 0.02f, \
    .Kd = 0.0f, \
    .max_out = 25000.0f, \
    .max_Int = 5000.0f, \
    .last_error = 0.0f, \
    .Int_error = 0.0f, \
    .time_const = 0.001f \
}

	extern PID_typedef gimbal_angle_pid;
	//角度误差计算
	float gimbal_angle_error(float target_angle,float current_angle);

	int16_t gimbal_angle_control(float target_angle);


	



	



#endif

