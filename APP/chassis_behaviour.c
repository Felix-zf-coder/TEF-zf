
#include "chassis_behaviour.h"
#define CHASSIS_MAX_W 2.0f
float motor_target_speeds[4] = {0,0,0,0};
int16_t motor_current[4] = {0,0,0,0};
float chassis_vx=0.0f;
float chassis_vy=0.0f;
float chassis_w=0.0f;

void chassis_speed_resolve(float vx,float vy,float w,float*motor_target_rpm )
{

    //vx前进后退，vy左右平移
    //


    //
    float wheel_circum = 2*PI*WHEEL_RADIUS;
    float motor_rpm_per_mps =(60.0f*GEAR_RATIO_M3508)/wheel_circum;//1m/s对应电机转速（rpm）
   float rot=w*(CHASSIS_LENGTH+CHASSIS_WIDTH);//旋转分量
    //
motor_target_rpm[0] = ( vx + vy + rot) * motor_rpm_per_mps;  // 电机1 左上
motor_target_rpm[1] = (-vx + vy +rot) * motor_rpm_per_mps;  // 电机2 右上
motor_target_rpm[2] = ( vx - vy + rot) * motor_rpm_per_mps;  // 电机3 左下
motor_target_rpm[3] = (-vx - vy + rot) * motor_rpm_per_mps;  // 电机4 右下
}



void remote_control_chassis(void)
{   

     float rc_ch3;
     float rc_ch4;

     float rc_ch1;
  
    //
     rc_ch3=(float)rc_ctrl.rc.Ch3;
     rc_ch4=(float)rc_ctrl.rc.Ch4;
     rc_ch1=(float)rc_ctrl.rc.Ch1;

    rc_ch3=(fabs(rc_ch3)<RC_DEAD_ZONE)?0.0f:rc_ch3;
    rc_ch4=(fabs(rc_ch4)<RC_DEAD_ZONE)?0.0f:rc_ch4;
    rc_ch1=(fabs(rc_ch1)<RC_DEAD_ZONE)?0.0f:rc_ch1;

    //
    chassis_vx=(rc_ch3/RC_CH_MAX)*CHASSIS_MAX_SPEED;
    chassis_vy=(rc_ch4/RC_CH_MAX)*CHASSIS_MAX_SPEED;
    chassis_w=(rc_ch1/RC_CH_MAX)*CHASSIS_MAX_W;

    //
     chassis_speed_resolve(chassis_vx,chassis_vy,chassis_w,motor_target_speeds);
  
    //
    calcuate_four_motors(); 
}


// 四电机PID控制器
PID_typedef motor_pid[4] = {
    C620_M3508_SPEED_PID_PARA,
    C620_M3508_SPEED_PID_PARA, 
    C620_M3508_SPEED_PID_PARA,
    C620_M3508_SPEED_PID_PARA
};



void calcuate_four_motors(void)
{
    //启动延时
 static uint32_t startup_count =0;
 if (startup_count<50)
 {
    startup_count++;
    can1_four(0,0,0,0);
    return;
 }
 
    // 计算四个电机的PID输出
    for(int i = 0; i < 4; i++) {
       if (motor_target_speeds[i]==0)
    {
        motor_pid[i].Int_error=0.f;
        motor_current[i]=0;
        continue;
    }
		
        //
        float error = motor_target_speeds[i] -  (float)LAUNCHER_CAN1[i].speed_rpm;
        //

        motor_current[i] = (int16_t)PID_Compute(&motor_pid[i], error);

    }
////       // 发送四个电机控制命令
  can1_four(motor_current[0],motor_current[1], motor_current[2],motor_current[3]);
}


