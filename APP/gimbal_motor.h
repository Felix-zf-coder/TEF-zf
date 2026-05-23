#ifndef __GIMBAL_MOTOR_H__
#define __GIMBAL_MOTOR_H__

#include "pid.h"
#include "stdint.h"

#define GIMBAL_ANGLE_PID_PARA \
{ \
    .Kp = 5.0f, \
    .Ki = 0.02f, \
    .Kd = 0.0f, \
    .max_out = 5000.0f, \
    .max_Int = 1000.0f, \
    .last_error = 0.0f, \
    .Int_error = 0.0f, \
    .time_const = 0.001f \
}

#define GIMBAL_SPEED_PID_PARA \
{ \
	.Kp         =  6.4f, \
	.Ki         = 0.2f, \
	.Kd         = 0.0f, \
	.max_out    = 25000.0f, \
	.max_Int    = 5000.0f, \
	.last_error =  0.0f, \
	.Int_error  =  0.0f, \
	.time_const =  0.001f, \
}
	extern PID_typedef gimbal_angle_pid;
  extern PID_typedef gimbal_speed_pid;

	//角度误差计算
	float gimbal_angle_error(float target_angle,float current_angle);

	int16_t gimbal_angle_control(float target_angle);

 #endif


