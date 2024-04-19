/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file my_init.c
 * @brief 程序的初始化
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
 
#include "my_init.h"

void mode_init(void)
{
	chassis_center.pid.loop_flag=POSITION_LOOP;
	chassis_motor1.pid.loop_flag=SPEED_LOOP;
	chassis_motor2.pid.loop_flag=SPEED_LOOP;
	chassis_motor3.pid.loop_flag=SPEED_LOOP;
	chassis_motor4.pid.loop_flag=SPEED_LOOP;
	
}
void all_init(void)
{
	motor_pid_init(&(chassis_center.pid));
	motor_pid_init(&(chassis_motor1.pid));
	motor_pid_init(&(chassis_motor2.pid));
	motor_pid_init(&(chassis_motor3.pid));
	motor_pid_init(&(chassis_motor4.pid));
	pid_init();
	
//	POWER_PID_Init(&p_pid);//功率
    DR16_Rx_Init();
	CAN1_Filter_Init();    //can
}

