 #include "gimbal_motor.h"
#include "main.h"
#include "pid.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_tim.h"
#include "vofa.h"
#include <stdint.h>
#include "tim.h"
#include "WT901.h"
#include <math.h>

#define  SA_THRESHOLD   300

/* Ch1 遥控控制 yaw 目标偏移（速度模式） */
#define  YAW_RC_CH_MAX        671.0f   /* Ch1 满杆量程（已减中点偏移） */
#define  YAW_RC_DEAD_ZONE     10.0f    /* Ch1 死区 */
#define  GIMBAL_YAW_MAX_RATE  120.0f   /* 满杆时 yaw 目标角速度 (deg/s) */

    //PID参数定义
	PID_typedef gimbal_angle_pid = GIMBAL_ANGLE_PID_PARA;
	PID_typedef gimbal_speed_pid = GIMBAL_SPEED_PID_PARA;
	PID_typedef gimbal_imu_yaw_pid = GIMBAL_IMU_YAW_PID_PARA;
static float imu_yaw_target = 0.0f;
static float imu_yaw_offset = 0.0f;
static volatile uint8_t angle_control_flag = 0U;
static uint8_t angle_divider = 0U;
static float imu_target_speed = 0.0f;
static int16_t imu_voltage_cmd = 0;
 float gimbal_angle_format(float angle)
	{
		while(angle>=8191.0f)
		{
			angle-=8191.0f;
		}
        while (angle<0.0f)
		{
			angle+=8191.0f;
		}
			return angle;
	}

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
 // 
 float gimbal_imu_yaw_error(float target_yaw, float current_yaw)
 {
     float error = target_yaw-current_yaw;
	 while (error>180.0f) 
	 { 
		error -=360.0f;
	 
	 }
	 while (error<-180.0f)
	  {
		
		error+=360.0f;
	 
	 }
	 return error;

 }

//

float gimbal_imu_yaw_control(float target_yaw,
                             float current_yaw)
{
    float yaw_error =
        gimbal_imu_yaw_error(target_yaw, current_yaw);

    return PID_Compute(&gimbal_imu_yaw_pid, yaw_error);
}

//
void gimbal_imu_yaw_start(void)
{
/*保存启动时的世界Yaw方向*/
imu_yaw_target=wit901c_data.angle[2];
/*清空PID历史数据*/
gimbal_speed_pid.last_error = 0.0f;
gimbal_speed_pid.Int_error = 0.0f;

imu_target_speed = 0.0f;
imu_yaw_offset = 0.0f;
imu_voltage_cmd = 0;
angle_control_flag = 0U;
angle_divider = 0U;
/*启动TIM2的20ms更新中断*/

if (HAL_TIM_Base_Start_IT(&htim2)!=HAL_OK)
 {
    Error_Handler();

}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        angle_divider++;

        if (angle_divider >= 20U)
        {
            angle_divider = 0U;
            angle_control_flag = 1U;
        }
    }
}

void gimbal_imu_yaw_task(void)
{
 

    if (wit901c_data.opened == 0U)
    {
        imu_target_speed = 0.0f;
        imu_voltage_cmd = 0;
        can1_one(0);
        return;
    }

    /* IMU角度外环：TIM2每20ms允许更新一次 */
    if (angle_control_flag != 0U)
    {
        angle_control_flag = 0U;

        /* Ch1 拨杆积分更新目标偏移：拨动时枪口转，回中后停在新位置 */
        float rc_ch1 = (float)rc_ctrl.rc.Ch1;
        if (fabsf(rc_ch1) > YAW_RC_DEAD_ZONE)
        {
            imu_yaw_offset += (rc_ch1 / YAW_RC_CH_MAX)
                              * GIMBAL_YAW_MAX_RATE * 0.020f;
        }

        imu_target_speed = gimbal_imu_yaw_control(
            imu_yaw_target + imu_yaw_offset,
            wit901c_data.angle[2]);

  
    }

    /* 速度内环：每次while循环都执行，没有延时 */
    float current_speed =
        (float)GIMBAL_CAN1[0].speed_rpm;

    float speed_error =
        imu_target_speed - current_speed;

    imu_voltage_cmd = (int16_t)PID_Compute(
        &gimbal_speed_pid,
        speed_error);

    can1_one(imu_voltage_cmd);

   float yaw_error = gimbal_imu_yaw_error(
    imu_yaw_target + imu_yaw_offset,
    wit901c_data.angle[2]);


        vofa_send(yaw_error,
                  imu_yaw_target + imu_yaw_offset,
                  wit901c_data.angle[2]);
  
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
		float voltage_cmd = PID_Compute(&gimbal_speed_pid, speed_error);

		return (int16_t)voltage_cmd;
	}





	float get_sa_target_speed(void)
	{
		if(rc_ctrl.rc.SA < -SA_THRESHOLD)
		{
			return vofa_target_speed;

		}
		else if (rc_ctrl.rc.SA > SA_THRESHOLD)
		{
			return -vofa_target_speed;
		}
		else 
		{
			return 0.0f;
		}

	}









