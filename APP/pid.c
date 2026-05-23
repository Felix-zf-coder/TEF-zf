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






