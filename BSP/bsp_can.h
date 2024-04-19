#ifndef __BSP_CAN_H
#define __BSP_CAN_H

#include "can.h"
#include "chassis_move.h"
#include "stm32f4xx_hal_can.h"

typedef struct
{
		uint32_t mailbox;               
		CAN_TxHeaderTypeDef TxMessage;  //发送结构体
		uint8_t payload[8];             //要发送的数据
}CANTxMsg_t;

typedef struct
{
		CAN_RxHeaderTypeDef RxMessage;  //接收结构体
		uint8_t payload[8];             //接收数据的存储位置
}CANRxMsg_t;                        
	
#define CAN_3508Motor1_ID         0x201
#define CAN_3508Motor2_ID         0x202
#define CAN_3508Motor3_ID         0x203
#define CAN_3508Motor4_ID         0x204
	
void CAN1_Filter_Init(void);
uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata);

#endif

