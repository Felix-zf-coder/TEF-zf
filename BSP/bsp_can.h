#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"  // 包含主头文件
#include <stdint.h>
//底盘3508四个电机数据结构体定义
typedef struct
{
    uint16_t ecd;           // 编码器值
    int16_t speed_rpm;      // 转速
    int16_t given_current;  // 电流
    uint8_t temperate;      // 温度
    int16_t last_ecd;       // 上次编码器值

} motor_measure_t;

//can1_3508数据解析宏
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


//云台6020两个电机数据结构体定义
typedef struct
{
    uint16_t angle;
    int16_t speed_rpm;
    int16_t torque_current;
    uint8_t temperate;
    int16_t last_angle;
}gm6020_measure_t;


#define get_gimbal_motor_measure(ptr,data)                          \
    {                                                               \
        (ptr)->last_angle = (ptr)->angle;                           \
        (ptr)->angle =(uint16_t)((data)[0]<<8 |(data)[1]);           \
        (ptr)->speed_rpm = (int16_t)((data)[2]<<8 |(data)[3]);      \
        (ptr)->torque_current = (int16_t)((data)[4]<<8 |(data)[5]); \
        (ptr)->temperate = (data)[6];                               \
    }



// 函数声明
void can_filter_init(void);  // CAN过滤器初始化

void can1_four(int16_t TR3508, int16_t LR3508, int16_t TL3508, int16_t BL3508);//can1发送4个电机数据
void can1_one(int16_t voltage);//can1发送1个电机数据
// 外部声明电机数据数组
extern motor_measure_t LAUNCHER_CAN1[4];  // CAN1: ID 0x201~0x204
extern gm6020_measure_t GIMBAL_CAN1[1];  // CAN1: ID 0x209
#endif // BSP_CAN_H
