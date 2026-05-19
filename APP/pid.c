#include "PID.h"
#include <math.h>

/** @function PID_Compute 普通PID计算函数
	* @parameter:     pid 普通PID句柄
	* @parameter:new_data 新误差
  */

  float PID_Compute(PID_typedef* pid, float new_error)
    {
	// 历史误差传递
	float last_error = pid->last_error;
	pid->last_error = new_error;

  
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


/** @function Low_Pass_PID_Compute 低通滤波PID计算函数
	* @parameter:      pid 低通滤波PID句柄
	* @parameter: new_data 新误差
  */

  float Low_Pass_PID_Compute(
	Low_Pass_PID_typedef* pid, 
	float new_error)
{
  // 对新误差进行滤波
	float low_pass_error = pid->forward * pid->last_error + (1-pid->forward) * new_error;
	

    // 历史误差传递
	float last_error = pid->last_error;
	pid->last_error = low_pass_error;
	
    // 积分计算与限制
	pid->Int_error += low_pass_error * pid->time_const;
	if(pid->Int_error > pid->max_Int)pid->Int_error = pid->max_Int;
	if(pid->Int_error < -pid->max_Int)pid->Int_error = -pid->max_Int;
	
    // PID反馈分量计算
	float out_p = pid->Kp * low_pass_error;
	float out_i = pid->Ki * pid->Int_error;
	float out_d = pid->Kd * (low_pass_error - last_error) / pid->time_const;
	
    // 反馈和
	float out = out_p + out_i + out_d;
    // 限制并输出
	return out > pid->max_out ? pid->max_out : (out < -pid->max_out ? -pid->max_out : out);
}


