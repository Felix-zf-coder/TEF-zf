 #include "gimbal_motor.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_control.h"
 
 
    //PID参数定义
	PID_typedef gimbal_angle_pid = GIMBAL_ANGLE_PID_PARA;
	PID_typedef gimbal_speed_pid = GIMBAL_SPEED_PID_PARA;
	//角度误差计算
	float gimbal_angle_error(float target_angle,float current_angle)
	{
		float error = target_angle - current_angle;
		if(error > 4095.0f)
		error -= 8191.0f;
	else if (error < -4095.0f)
	  error += 8191.0f;
	 return error;
	}
//角度闭环
	int16_t gimbal_angle_control(float target_angle)
	{
		float  current_angle = (float)GIMBAL_CAN1[0].angle;
		float  current_speed = (float)GIMBAL_CAN1[0].speed_rpm;
		
		// 外环：角度误差 → 目标转速
		float  angle_error = gimbal_angle_error(target_angle,current_angle);
		float  target_speed = PID_Compute(&gimbal_angle_pid, angle_error);
		
		 // 内环：转速误差 → 电压
		float speed_error = target_speed - current_speed;
		float out = PID_Compute(&gimbal_speed_pid, speed_error);

		return (int16_t)out;
	}













