#ifndef __DR16_H
#define __DR16_H

#include "dma.h"
#include "usart.h"
#include "chassis_move.h"

typedef struct
{
	struct
	{ 
		 int ch0;
		 int ch1;
		 int ch2;
		 int ch3;
		 char s1;
		 char s2;
		
	}rc;
}RC_Ctl_t;

#define Numeric_Conversion (15535.0/660.0)*3 //遥控器数值转换为速度

#define REMOTE_MAX 660
#define REMOTE_MIN -660
#define REMOTE_MID 0


#define SBUS_RX_BUF_NUM 36u
#define RC_FRAME_LENGTH 18u

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern RC_Ctl_t RC_Ctl;


void Get_DR16_Data(uint8_t *sbus_rx_buffer,RC_Ctl_t* RC_Ctl);//数据包解析函数
const RC_Ctl_t *get_remote_control_point(void);
void DR16_Rx_Init(void);
void remote_control(void);//遥控器通道值转化以及模式选择

#endif
