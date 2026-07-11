#include "WT901.h"
#include <stdint.h>

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
























