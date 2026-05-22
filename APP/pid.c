#include "PID.h"
#include <math.h>

#include "bsp_can.h"


  float PID_Compute(PID_typedef* pid, float new_error)
    {
	// 历史误差传递
	float last_error = pid->last_error;
	pid->last_error = new_error;
	// 积分误差计算
	if (fabsf(new_error)<50.0f)
	{
		pid->Int_error+=new_error*pid->time_const;
		if(pid->Int_error > pid->max_Int)pid->Int_error = pid->max_Int;
	    if(pid->Int_error < -pid->max_Int)pid->Int_error = -pid->max_Int;
	}
	else {
		
		pid->Int_error=0.0f;
	}
    // PID反馈分量计算
	float out_p = pid->Kp * new_error;
	float out_i = pid->Ki * pid->Int_error;
	float out_d = pid->Kd * (new_error - last_error) / pid->time_const;
	
    // 反馈和
	float out = out_p + out_i + out_d;
	   // 添加输出限制和返回值
    if(out > pid->max_out) out = pid->max_out;
    if(out < -pid->max_out) out = -pid->max_out;
    
    return out;
    }


	//PID参数定义
	PID_typedef gimbal_angle_pid = GIMBAL_ANGLE_PID_PARA;
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
		float error = gimbal_angle_error(target_angle,current_angle);
		int16_t out = (int16_t)PID_Compute(&gimbal_angle_pid,error );
		return (int16_t)out;
	}




