/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file weak.c
 * @brief RTOS的任务函数
 * @note 笔记
 * @history 2024/4/14
 *
 @verbatim 
 ==============================================================================
 1、底盘移动控制任务
 ==============================================================================
 @endverbatim
 *****************************东莞理工学院ACE实验室 *****************************
 */
#include "weak.h"

void Chassis_move(void *argument)//CAN发送，20ms发送一次
{
    //限幅测试
//    SET_SPEED(660,0,10);
//    chassis_control_order.chassis_mode = CHASSIS_NORMAL;
    while(1)
    {
        //启用遥控器
        remote_control();
		chassis_move();
        osDelay(20);
    } 
}
