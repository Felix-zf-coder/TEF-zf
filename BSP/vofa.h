#ifndef __VOFA_H__
#define __VOFA_H__

#include "main.h"
#include "pid.h"

extern float vofa_target_speed;  // 添加这行

void vofa_send(float ch1, float ch2, float ch3);
void vofa_init(void);

#endif