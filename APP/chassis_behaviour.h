#ifndef __CHASSIS_BEHAVIOUR_H__
#define __CHASSIS_BEHAVIOUR_H__

#include "main.h"
#include "remote_control.h"
#include "pid.h"
#include "bsp_can.h"
#include "gimbal_motor.h"
#include "math.h"

#define GEAR_RATIO_M3508 19.2032086f
#define WHEEL_RADIUS 0.075f
#define CHASSIS_WIDTH 0.425f
#define CHASSIS_LENGTH 0.500f
#define CHASSIS_MAX_SPEED 0.8f
#define RC_CH_MAX 671.0f
#define RC_DEAD_ZONE 10.0f
#define PI 3.1415926535f



#define C620_M3508_SPEED_PID_PARA \
{ \
	.Kp         =  6.4f, \
	.Ki         = 0.2f, \
	.Kd         = 0.0f, \
	.max_out    = 4000.0f, \
	.max_Int    = 5000.0f, \
	.last_error =  0.0f, \
	.Int_error  =  0.0f, \
	.time_const =  0.020f, \
}

void remote_control_chassis(void);
void chassis_speed_resolve(float vx,float vy,float w,float*motor_target_rpm );
void calcuate_four_motors(void);

#endif








