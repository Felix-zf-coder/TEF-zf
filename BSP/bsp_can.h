#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "main.h"  // 鍖呭惈涓诲ご鏂囦欢
#include <stdint.h>
//搴曠洏3508鍥涗釜鐢垫満鏁版嵁缁撴瀯浣撳畾涔?
typedef struct
{
    uint16_t ecd;           // 缂栫爜鍣ㄥ€?
    int16_t speed_rpm;      // 杞€?
    int16_t given_current;  // 鐢垫祦
    uint8_t temperate;      // 娓╁害
    int16_t last_ecd;       // 涓婃缂栫爜鍣ㄥ€?

} motor_measure_t;

//can1_3508鏁版嵁瑙ｆ瀽瀹?
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }


//浜戝彴6020涓や釜鐢垫満鏁版嵁缁撴瀯浣撳畾涔?
typedef struct
{
    uint16_t angle;//编码器的值
    int16_t speed_rpm;//每秒转速
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



// 鍑芥暟澹版槑
void can_filter_init(void);  // CAN杩囨护鍣ㄥ垵濮嬪寲

void can1_four(int16_t TR3508, int16_t LR3508, int16_t TL3508, int16_t BL3508);//can1鍙戦€?涓數鏈烘暟鎹?
void can1_one(int16_t voltage);//can1鍙戦€?涓數鏈烘暟鎹?
// 澶栭儴澹版槑鐢垫満鏁版嵁鏁扮粍
extern motor_measure_t LAUNCHER_CAN1[4];  // CAN1: ID 0x201~0x204
extern gm6020_measure_t GIMBAL_CAN1[1];  // CAN1: ID 0x209
#endif // BSP_CAN_H
