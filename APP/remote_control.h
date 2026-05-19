#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "main.h"
#include "string.h"


#define SBUS_RX_BUF_NUM 50u

#define RC_FRAME_LENGTH 25u

#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)

#define RC_SW_UP                ((uint16_t)-671)
#define RC_SW_MID               ((uint16_t)0)
#define RC_SW_DOWN              ((uint16_t)671)


typedef   struct {
    uint16_t w : 1;      // bit 0: W��
    uint16_t s : 1;      // bit 1: S��
    uint16_t a : 1;      // bit 2: A��
    uint16_t d : 1;      // bit 3: D��
    uint16_t shift : 1;  // bit 4: Shift��
    uint16_t ctrl : 1;   // bit 5: Ctrl��
    uint16_t q : 1;      // bit 6: Q��
    uint16_t e : 1;      // bit 7: E��
    uint16_t r : 1;      // bit 8: R��
    uint16_t f : 1;      // bit 9: F��
    uint16_t g : 1;      // bit 10: G��
    uint16_t z : 1;      // bit 11: Z��
    uint16_t x : 1;      // bit 12: X��
    uint16_t c : 1;      // bit 13: C��
    uint16_t v : 1;      // bit 14: V��
    uint16_t b : 1;      // bit 15: B��
} keyboard_state_t;


typedef  struct
{
        __packed struct
        {
            int16_t Start;	
	
						int16_t Ch1;//�Һ�
						int16_t Ch2;//����
						int16_t Ch3;//����
						int16_t Ch4;//���

						int16_t SA;
						int16_t SB;
						int16_t SC;
						int16_t SD;
						
						int16_t SE;
						int16_t SF;
						int16_t SG;
						int16_t SH;
		
						int16_t LD;
						int16_t RD;
						int16_t LS;
						int16_t RS;
        } rc;
				
				__packed struct
				{
						int16_t mouse_x; //���x���ƶ��ٶȣ���ֵ��ʶ�����ƶ� 

						int16_t mouse_y; //���y���ƶ��ٶȣ���ֵ��ʶ�����ƶ�
						int16_t mouse_z; //�������ƶ��ٶȣ���ֵ��ʶ������

						int8_t left_button_down; //�������Ƿ��£�0Ϊδ���£�1Ϊ����
						int8_t right_button_down; //����Ҽ��Ƿ��£�0Ϊδ���£�1Ϊ���� 

						keyboard_state_t keyboard; //���̰�����Ϣ��ÿ��bit��Ӧһ��������0Ϊδ���£�1Ϊ����

						uint16_t reserved; //����λ
				} key_mouse;
        

} RC_ctrl_t;

extern RC_ctrl_t rc_ctrl;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;



void remote_control_init(void);
void USART1_IRQHandler123(void);
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);







#endif
