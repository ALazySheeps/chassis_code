/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file bsp_can.c
 * @brief bsp层CAN功能
 * @note 笔记
 * @history 2024/4/14
 *
 @verbatim 
 ==============================================================================
 用于建立一个框，来说明整个文件的功能。
 ==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */
#include "bsp_can.h"

static void read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan,int16_t *adata);//分位读取数据函数
static void get_motor_data(MOTOR_t *motor, int16_t angle, float speed, int16_t current);

//筛选器配置
void CAN1_Filter_Init(void)
{
		CAN_FilterTypeDef CAN1_FilerConf;
	
        //配置fifo0
        CAN1_FilerConf.FilterIdHigh = 0;					   //具体Id要求髿16使
		CAN1_FilerConf.FilterIdLow = 0;					   //具体Id要求使16使
		CAN1_FilerConf.FilterMaskIdHigh = 0;                
		CAN1_FilerConf.FilterMaskIdLow = 0;                 
		CAN1_FilerConf.FilterFIFOAssignment=CAN_FILTER_FIFO0;  //筛鿉器接收到的报文放入到FIFO0中，即为接收邮箱0
		CAN1_FilerConf.FilterActivation=ENABLE;                //筛鿉器使能（弿启）
		CAN1_FilerConf.FilterMode=CAN_FILTERMODE_IDMASK;       //筛鿉器掩码模弿
		CAN1_FilerConf.FilterScale=CAN_FILTERSCALE_32BIT;      //掩码甿32位表礿
		/*
			can1的筛选器组鿉拿0-13。如CAN1_FilerConf.FilterBank=0;	
			can2的筛选器组鿉拿14-27。如CAN1_FilerConf.SlaveStartFilterBank=14;	
		*/
		CAN1_FilerConf.FilterBank=0;							
		//CAN1_FilerConf.SlaveStartFilterBank=14;
		
		/*	此处&hcan1指明是can1的筛选器配置，但实际上can1和can2的筛选器都配置好了㿂因为两个can是共用的〿
			这是因为STM32的双路CAN共用过滤器组＿
			而且过滤器组寄存器与CAN1配置寄存器在物理上是挨着的，HAL库将这些寄存器合并在丿个结构里访问迌已〿
			下面通过调用 "HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)" 配置can筛鿉器即可生效㿿
			无需再调用HAL_CAN_ConfigFilter(&hcan2,&CAN1_FilerConf)
		*/
		if(HAL_CAN_ConfigFilter(&hcan1,&CAN1_FilerConf)!=HAL_OK)
		{
				Error_Handler();
		}
        
        HAL_CAN_Start(&hcan1);//打开CAN
        
        if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)//开启中断
        {
            Error_Handler();
        }
}

int flag9=0;

uint8_t Can_Tx_Message(CAN_HandleTypeDef *hcan,uint8_t *mdata)//CAN发送
{
	int i;
	uint8_t status;
    CANTxMsg_t TxMsg;
	for(i=0;i<8;i++)
        TxMsg.payload[i]=mdata[i];    //发送的数据
    
	if(hcan==&hcan1)
	{
		flag9=1;
		TxMsg.TxMessage.StdId=0x200;    		
        TxMsg.TxMessage.ExtId=0;
        TxMsg.TxMessage.IDE=CAN_ID_STD; 				
        TxMsg.TxMessage.DLC=8;						
        TxMsg.TxMessage.RTR=CAN_RTR_DATA;			
        TxMsg.TxMessage.TransmitGlobalTime=DISABLE;  //默认DISABLE
	}
    else if(hcan == &hcan2)
    {
        
    }
    //向空闲发送邮箱添加消息
	status=HAL_CAN_AddTxMessage(hcan,&TxMsg.TxMessage,TxMsg.payload,&TxMsg.mailbox);
    
	return status; //是否添加成功标志位
}

int flag4=0;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//CAN接收回调函数
{
	flag4=1;
	CANRxMsg_t RxMsg;
    int16_t speed,current,angle;
    
    int16_t adata[4];//将接收的数据存储在adata里
	if(hcan==&hcan1)//CAN1电调对应回调
	{
        /*接收CAN，数据存储在RxMsg.payload里面*/
		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxMsg.RxMessage,RxMsg.payload);
        /*处理payload里面的数据，合并高八位低八位，存储在adata数组中*/
		read_motor_data(RxMsg.payload,&hcan1,adata);
        //memset(adata,0,sizeof(adata));
		angle=adata[0];
		speed=adata[1];
		current=adata[2];
        
		switch(RxMsg.RxMessage.StdId)
		{
			case CAN_3508Motor1_ID:
				get_motor_data(&chassis_motor1,angle,(float)speed,current);
			break;
			case CAN_3508Motor2_ID:
				get_motor_data(&chassis_motor2,angle,(float)speed,current);
			break;
			case CAN_3508Motor3_ID:
				get_motor_data(&chassis_motor3,angle,(float)speed,current);
			break;
			case CAN_3508Motor4_ID:
				get_motor_data(&chassis_motor4,angle,(float)speed,current);
			break;
			default:break;
		}
	}
	else if(hcan == &hcan2)
    {
        
    }
}

static void read_motor_data(uint8_t *rxdata,CAN_HandleTypeDef *hcan,int16_t *adata)//分位读取数据函数
{
	if(hcan==&hcan1)
	{
		adata[0]= (rxdata[0]<<8)|rxdata[1];//机械角度
		adata[1]= (rxdata[2]<<8)|rxdata[3];//转速
		adata[2]= (rxdata[4]<<8)|rxdata[5];//电流
		adata[3]= (rxdata[6]<<8)|rxdata[7];//电机温度
	}
    else if(hcan == &hcan2)
    {
        
    }
}

//处理接收的can消息：角度、速度、电流
static void get_motor_data(MOTOR_t *motor, int16_t angle, float speed, int16_t current)
{
//	motor->last_angle = motor->actual_angle;//当前角度更新
//	motor->actual_angle = angle;
    
	motor->actual_speed = 0.5f*(speed + motor->last_speed);
    
	motor->pid.speed_loop.vpid.actual_speed=motor->actual_speed;
    
	motor->last_speed = speed;
    
//	if(motor->actual_angle - motor->last_angle > 4096)
//		motor->round_cnt--;
//	else if (motor->actual_angle - motor->last_angle < -4096)
//		motor->round_cnt++;
//	motor->total_angle = motor->round_cnt * 8192 + motor->actual_angle;

}

