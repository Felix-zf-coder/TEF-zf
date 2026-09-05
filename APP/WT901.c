#include "WT901.h"
#include "usart.h"
#include <stdint.h>
volatile uint8_t wit_dma_status = 0xFF;
volatile uint32_t wit_rx_count = 0;
volatile uint32_t wit_ok_count = 0;
//
uint8_t Wit901c_Data_Process
(uint8_t*data, Wit901C *wit901c_data)
{
    uint8_t sum=0;
    if(data[0]!=WIT901C_HEAD) return 0x01;
switch (data[1]){
    case ACC_HEAD:case ANGVEL_HEAD:case ANGLE_HEAD:
    for(uint8_t i=0;i<10;i++)
    sum+=data[i];
if(sum!=data[10]) return 0x02;
break;
default:
return 0x02;

}


//
switch(data[1]){

         case ACC_HEAD:
    for(uint8_t i=0; i<3;i++)
    wit901c_data->acc[i]=16.0f*RECIPROCAL32768*(short)((short)data[i*2+3]<<8|data[i*2+2]);
	    break;
        

        case ANGVEL_HEAD: 
    for(uint8_t i=0; i<3; i++)
    wit901c_data->angvel[i] = 2000.0f * RECIPROCAL32768 * (short)((short)data[i*2+3]<<8 | data[i*2+2]);
		break;


		case ANGLE_HEAD:
	for(uint8_t i=0; i<3; i++)
    wit901c_data->angle[i] = 180.0f * RECIPROCAL32768 * (short)((short)data[i*2+3]<<8 | data[i*2+2]);
	    break;

	default:
		return 0x03; // TYPE位异常错误

}
wit901c_data->opened =0x01;
return 0x00;
}
static uint8_t wit901c_rx_buf[TOP_BUF_LEN];
Wit901C wit901c_data = {0};

void Wit901c_Init(void)
{
    wit_dma_status = HAL_UART_Receive_DMA(&huart6,
                                          wit901c_rx_buf,
                                          TOP_BUF_LEN);
}

void Wit901c_RxCplt(void)
{
    wit_rx_count++;

    if(Wit901c_Data_Process(wit901c_rx_buf,
                            &wit901c_data) == 0)
    {
        wit_ok_count++;
    }
}























