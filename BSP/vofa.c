#include "vofa.h"
#include "usart.h"
#include "gimbal_motor.h"
#include "stdio.h"
#include "string.h"
#include <stdint.h>

float  vofa_target_speed =50.0f;

// 全局变量，用于调试
volatile uint8_t rx_received = 0;
// 解析指令
static void parse_cmd(char *buf)
{
    float kp, ki, kd;
    float spd;

    // 解析PID: "pid:30.0,0.1,0.0"
    if(sscanf(buf, "pid:%f,%f,%f", &kp, &ki, &kd) == 3)
    {
        gimbal_speed_pid.Kp = kp;
        gimbal_speed_pid.Ki = ki;
        gimbal_speed_pid.Kd = kd;
    }
    // 解析目标速度: "spd:200.0"
    else if(sscanf(buf, "spd:%f", &spd) == 1)
    {
        vofa_target_speed = spd;
    }
}

// 发送
void vofa_send(float ch1, float ch2, float ch3)
{
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f\n", ch1, ch2, ch3);
    HAL_UART_Transmit(&huart7, (uint8_t*)buf, len, 10);
}

// 接收缓冲
static uint8_t rx_byte;

void vofa_init(void)
{
    HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == UART7)
    {
        rx_received++;  // 每收到一个字节就加1
        static char line[64];
        static uint8_t idx = 0;

        char c = (char)rx_byte;
        if(c == '\n')
        {
            line[idx] = '\0';
            parse_cmd(line);
            idx = 0;
        }
        else if(idx < 63)
        {
            line[idx++] = c;
        }
        HAL_UART_Receive_IT(&huart7, &rx_byte, 1);
    }
}