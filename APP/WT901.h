#ifndef __WT901_H__
#define __WT901_H__

#include <stdint.h>
#include <sys/types.h>
#define  TOP_BUF_MAX_LEN 64
#define TOP_BUF_LEN 11
//Wit901
typedef struct __Wit901C
{
    float acc[3],angvel[3],angle[3];
    uint8_t opened;

}Wit901C;


#define WIT901C_HEAD (0X55)
#define ACC_HEAD (0X51)
#define ANGVEL_HEAD (0X52)
#define ANGLE_HEAD (0X53)
#define RECIPROCAL32768 (3.0517578125e-05f)

//
uint8_t Wit901c_Data_Process(
    uint8_t*data,
    Wit901C*wit901c_data);



#endif

