#ifndef __GIMBAL_MOTOR_H__
#define __GIMBAL_MOTOR_H__
#include "main.h"
#include "pid.h"
#include "stdint.h"
#define GIMBAL_ZERO_ANGLE 6852.0f
#define GIMBAL_ANGLE_90 2408.0f
#define GIMBAL_ANGLE_PID_PARA \
{ \
    .Kp = 0.15f, \
    .Ki = 0.0f, \
    .Kd = 0.0f, \
    .max_out = 50.0f, \
    .max_Int = 1000.0f, \
    .last_error = 0.0f, \
    .Int_error = 0.0f, \
    .time_const = 0.001f \
}

#define GIMBAL_SPEED_PID_PARA \
{ \
	.Kp         =65.0f, \
	.Ki         =1.4f, \
	.Kd         = 0.0f, \
	.max_out    = 24000.0f, \
	.max_Int    = 5000.0f, \
	.last_error =  0.0f, \
	.Int_error  =  0.0f, \
	.time_const =  0.001f, \
}
//65,1.4


  extern PID_typedef gimbal_angle_pid;
  extern PID_typedef gimbal_speed_pid;

	//角度误差计算
	float gimbal_angle_error(float target_angle,float current_angle);
	float gimbal_angle_format(float angle);
	int16_t gimbal_angle_control(float target_angle);
    float get_sa_target_speed(void);
 #endif


