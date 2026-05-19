#ifndef __PID_H__
#define __PID_H__
/** @typedef PID_typedef 普通PID句柄结构体
	* @parameter:   Kp-Ki-Kd 3个PID参数
	* @parameter:    max_out 最大反馈限制
	* @parameter:    max_Int 最大积分限制
	*	@parameter: last_error 上次误差
	* @parameter:  Int_error 积累误差
	* @parameter: time_const 时间间隔
  */

  typedef struct __PID_typedef{
	float Kp, Ki, Kd, max_out, max_Int, last_error, Int_error, time_const;
}PID_typedef;

/** @function PID_Compute 普通PID计算函数
	* @parameter:      pid 普通PID句柄
	* @parameter: new_data 新误差
  */

  float PID_Compute(
	PID_typedef* pid, 
	float new_error);

    /** @typedef Low_Pass_PID_typedef 低通滤波PID句柄结构体
* @parameter:    forward 前向系数,为0时关闭低通滤波，最大值为1
	* @parameter:   Kp-Ki-Kd 3个PID参数
	* @parameter:    max_out 最大反馈限制
	* @parameter:    max_Int 最大积分限制
	*	@parameter: last_error 上次误差
	* @parameter:  Int_error 积累误差
	* @parameter: time_const 时间间隔
  */

  typedef struct __Low_Pass_PID_typedef{
	float forward, Kp, Ki, Kd, max_out, max_Int, last_error, Int_error, time_const;
}Low_Pass_PID_typedef;

/** @function Low_Pass_PID_Compute 低通滤波PID计算函数
	* @parameter:      pid 低通滤波PID句柄
	* @parameter: new_data 新误差
  */
float Low_Pass_PID_Compute(
	Low_Pass_PID_typedef* pid, 
	float new_error);

#endif

