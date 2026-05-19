#include "bsp_can.h"
#include "can.h"
#include "stm32f4xx_hal_can.h"
#include <stdint.h>


static CAN_TxHeaderTypeDef Launcher_tx_message; //底盘发送报文头
static uint8_t   Launcher_can_send_data[8];//底盘发送数据

static CAN_TxHeaderTypeDef GIMBAL_tx_message; //云台发送报文头
static uint8_t GIMBAL_can_send_data[8];//云台发送数据

//CAN过滤器初始化
void can_filter_init(void)
{

    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);


    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

}

// 底盘四个电机控制(电流控制，需要电调支持)
void can1_four(int16_t TR3508, int16_t LR3508, int16_t TL3508, int16_t BL3508)
{
    Launcher_tx_message.StdId = 0x200;  // 标准ID 0x200
    Launcher_tx_message.IDE = CAN_ID_STD;  // 标准帧
    Launcher_tx_message.RTR = CAN_RTR_DATA;  // 数据帧
    Launcher_tx_message.DLC = 0x08;  // 数据长度8字节
    Launcher_can_send_data[0] = (TR3508 >> 8);  // TR电机高8位
    Launcher_can_send_data[1] = TR3508;          // TR电机低8位
    Launcher_can_send_data[2] = (LR3508 >> 8);   // LR电机高8位
    Launcher_can_send_data[3] = LR3508;          // LR电机低8位
    Launcher_can_send_data[4] = (TL3508 >> 8);   // TL电机高8位
    Launcher_can_send_data[5] = TL3508;          // TL电机低8位
    Launcher_can_send_data[6] = (BL3508 >> 8);   // BL电机高8位
    Launcher_can_send_data[7] = BL3508;          // BL电机低8位
    HAL_CAN_AddTxMessage(&hcan1, &Launcher_tx_message, Launcher_can_send_data, 0);  // CAN1发送四个电机数据
}


// 云台两个电机控制
void can1_one(int16_t voltage)
{
    GIMBAL_tx_message.StdId =0x2FF;  
    GIMBAL_tx_message.IDE = CAN_ID_STD;
    GIMBAL_tx_message.RTR = CAN_RTR_DATA;
    GIMBAL_tx_message.DLC =0x08;
    GIMBAL_can_send_data[0] = (voltage >> 8);
    GIMBAL_can_send_data[1] = voltage;
    GIMBAL_can_send_data[2] = 0;
    GIMBAL_can_send_data[3] = 0;
    GIMBAL_can_send_data[4] = 0;
    GIMBAL_can_send_data[5] = 0;  
    GIMBAL_can_send_data[6] = 0;
    GIMBAL_can_send_data[7] = 0;  
    HAL_CAN_AddTxMessage(&hcan1,&GIMBAL_tx_message,GIMBAL_can_send_data,0); // CAN1发送两个电机数据                
}





// 外部声明电机数据数组
 motor_measure_t LAUNCHER_CAN1[4]; // CAN1: ID 0x201~0x204

gm6020_measure_t GIMBAL_CAN1[1]; // CAN1: ID 0x209


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) 
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);

    // CAN1 的数据
    if (hcan->Instance == CAN1) 

   {
  
     //can1_3508数据解析
        switch (rx_header.StdId) {
      
        case 0x201:
            get_motor_measure(&LAUNCHER_CAN1[0], rx_data);
            break;
        case 0x202:
            get_motor_measure(&LAUNCHER_CAN1[1], rx_data);
            break;
        case 0x203:
            get_motor_measure(&LAUNCHER_CAN1[2], rx_data);
            break;
        case 0x204:
            get_motor_measure(&LAUNCHER_CAN1[3], rx_data);
            break;

        case 0x209:
            get_gimbal_motor_measure(&GIMBAL_CAN1[0], rx_data);
            break;
            default:
            return; // 如果不是预期的ID，直接返回
        }

    
}

    
    
}


