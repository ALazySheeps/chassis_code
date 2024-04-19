/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file MyDR16.c
 * @brief 遥控器接收和控制
 * @note 
 * @history 2024/4/14
 *
 @verbatim 
 ==============================================================================
 用于建立一个框，来说明整个文件的功能。
 ==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */
#include "MyDR16.h"

#define RC_huart    huart3
#define RC_UART		USART3
#define RC_dma		hdma_usart3_rx

static uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM]; //缓冲区
RC_Ctl_t RC_Ctl = {0};

void remote_control(void);
static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
uint8_t RC_data_is_error(void);
static void RC_restart(uint16_t dma_buf_num);//重新开启接收

//void DR16_Rx_Init(void)//--
//{
//    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE); //使能IDLE中断
//    HAL_UART_Receive_DMA(&huart3,Sbus_rx_buffer,BUFFER_SIZE);//打开DMA接收（DMA准备）
//}

//主函数初始化调用
void DR16_Rx_Init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);//使用静态变量
}

static void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//enable the DMA transfer for the receiver request
	//使能 DMA 串口接收
	SET_BIT(RC_huart.Instance->CR3, USART_CR3_DMAR);
	//enalbe idle interrupt
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&RC_huart, UART_IT_IDLE);
	//disable DMA
	//失效 DMA
	__HAL_DMA_DISABLE(&RC_dma);

	while(RC_dma.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&RC_dma);
	}

	RC_dma.Instance->PAR = (uint32_t) & (RC_UART->DR);
	//memory buffer 1
	//内存缓冲区 1
	RC_dma.Instance->M0AR = (uint32_t)(rx1_buf);
	//memory buffer 2
	//内存缓冲区 2
	RC_dma.Instance->M1AR = (uint32_t)(rx2_buf);
	//data length
	//数据长度
	RC_dma.Instance->NDTR = dma_buf_num;
	//enable double memory buffer
	//使能双缓冲区
	SET_BIT(RC_dma.Instance->CR, DMA_SxCR_DBM);
	//enable DMA
	//使能 DMA
	__HAL_DMA_ENABLE(&RC_dma);
}

//外部调用
const RC_Ctl_t *get_remote_control_point(void)
{
    return &RC_Ctl;
}

//判断遥控器数据是否出错，
uint8_t RC_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_Ctl.rc.ch0 > 1684)
    {
        goto error;
    }
    if (RC_Ctl.rc.ch1 > 1684)
    {
        goto error;
    }
    if (RC_Ctl.rc.ch2 > 1684)
    {
        goto error;
    }
    if (RC_Ctl.rc.ch3 > 1684)
    {
        goto error;
    }
    if (RC_Ctl.rc.s1 == 0)
    {
        goto error;
    }
    if (RC_Ctl.rc.s2 == 0)
    {
        goto error;
    }
    return 0;

error:
    RC_Ctl.rc.ch0 = 0;
    RC_Ctl.rc.ch1 = 0;
    RC_Ctl.rc.ch2 = 0;
    RC_Ctl.rc.ch3 = 0;
    RC_Ctl.rc.s1 = 1;
    RC_Ctl.rc.s2 = 1;
    return 1;
}


//数据包解析函数
void Get_DR16_Data(uint8_t *sbus_rx_buffer,RC_Ctl_t* RC_Ctl)
{
    RC_Ctl->rc.ch0 = (sbus_rx_buffer[0]| (sbus_rx_buffer[1] << 8)) & 0x07ff; 
    RC_Ctl->rc.ch0 -= 1024;                                          //设置范围为【-660~660】
	RC_Ctl->rc.ch1 = ((sbus_rx_buffer[1] >> 3) | (sbus_rx_buffer[2] << 5)) & 0x07ff;   
    RC_Ctl->rc.ch1 -= 1024;    
	RC_Ctl->rc.ch2 = ((sbus_rx_buffer[2] >> 6) | (sbus_rx_buffer[3] << 2) | (sbus_rx_buffer[4] << 10)) & 0x07ff;          
	RC_Ctl->rc.ch2 -= 1024;
    RC_Ctl->rc.ch3 = ((sbus_rx_buffer[4] >> 1) | (sbus_rx_buffer[5] << 7)) & 0x07ff;           
	RC_Ctl->rc.ch3 -= 1024;
    RC_Ctl->rc.s1  = ((sbus_rx_buffer[5] >> 4)& 0x000C) >> 2;                           
	RC_Ctl->rc.s2  = ((sbus_rx_buffer[5] >> 4)& 0x0003);
    
    if(RC_Ctl->rc.ch0 <= 5 && RC_Ctl->rc.ch0 >= -5)
        RC_Ctl->rc.ch0 = 0;
    if(RC_Ctl->rc.ch1 <= 5 && RC_Ctl->rc.ch1 >= -5)
        RC_Ctl->rc.ch1 = 0;
    if(RC_Ctl->rc.ch2 <= 5 && RC_Ctl->rc.ch2 >= -5)
        RC_Ctl->rc.ch2 = 0;
    if(RC_Ctl->rc.ch3 <= 5 && RC_Ctl->rc.ch3 >= -5)
        RC_Ctl->rc.ch3 = 0;
    
    return;
}

void USART3_IRQHandler(void)
{
	if(RC_huart.Instance->SR & UART_FLAG_RXNE)//接收到数据
	{
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
	}
	else if(RC_UART->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&RC_huart);
		if ((RC_dma.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			/* Current memory buffer used is Memory 0 */
			//disable DMA
			//失效 DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//重新设定数据长度
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 1
			//设定缓冲区 1
			RC_dma.Instance->CR |= DMA_SxCR_CT;
			//enable DMA
			//使能 DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				Get_DR16_Data(sbus_rx_buf[0], &RC_Ctl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
		else
		{
			/* Current memory buffer used is Memory 1 */
			//disable DMA
			//失效 DMA
			__HAL_DMA_DISABLE(&RC_dma);
			//get receive data length, length = set_data_length - remain_length
			//获取接收数据长度,长度 = 设定长度 - 剩余长度
			this_time_rx_len = SBUS_RX_BUF_NUM - RC_dma.Instance->NDTR;
			//reset set_data_lenght
			//重新设定数据长度
			RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
			//set memory buffer 0
			//设定缓冲区 0
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			//enable DMA
			//使能 DMA
			__HAL_DMA_ENABLE(&RC_dma);
		
			if(this_time_rx_len == RC_FRAME_LENGTH)
			{
				//处理遥控器数据
				Get_DR16_Data(sbus_rx_buf[1], &RC_Ctl);
				if(RC_data_is_error()==1)
				{
					RC_restart(SBUS_RX_BUF_NUM);
				}
			}
		}
	}
}

static void RC_restart(uint16_t dma_buf_num)//重新开启接收
{
	//disable UART
	__HAL_UART_DISABLE(&RC_huart);
	//disable DMA
	__HAL_DMA_DISABLE(&RC_dma);
	//reset set_data_lenght
	RC_dma.Instance->NDTR = SBUS_RX_BUF_NUM;
	__HAL_UART_CLEAR_IDLEFLAG(&RC_huart);
	__HAL_DMA_CLEAR_FLAG(&RC_dma,DMA_FLAG_TCIF2_6);
	// DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	//ensable UART
	__HAL_UART_ENABLE(&RC_huart);
	//ensable DMA
	__HAL_DMA_ENABLE(&RC_dma);
}

void remote_control(void)//遥控器通道值转化以及模式选择
{
		chassis_control_order.vx_set = (RC_Ctl.rc.ch1 * Numeric_Conversion);//右边摇杆左右
		chassis_control_order.vy_set = (RC_Ctl.rc.ch0 * Numeric_Conversion);//右边摇杆前后
		chassis_control_order.wz_set = (RC_Ctl.rc.ch2 * 100 / 660);//左边摇杆左右（控制旋转）
	
	switch(RC_Ctl.rc.s1)                                //S1（左拨杆）
	{
		case 3: {chassis_control_order.chassis_mode=CHASSIS_NORMAL;} break;//正常模式
		case 1: {chassis_control_order.chassis_mode=CHASSIS_REMOTE_CLOSE;} break;//底盘锁死模式
		case 2: {chassis_control_order.chassis_mode=CHASSIS_SPIN;} break;//小陀螺模式
		default:break;
	}
}

