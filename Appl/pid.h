#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include <stdlib.h>

//速度环宏定义
#define CHASSIS_Integral_max         2000.0            //抗积分饱和
#define CHASSIS_IntegralSeparation   500.0             //积分分离
#define CHASSIS_vPID_max             5000.0            //输出限幅

#define FOLLOW_Integral_max          500// 3000
#define FOLLOW_IntegralSeparation    15//20f
#define FOLLOW_vPID_max              10500
#define AVERAGE                      10              //平均误差数组长度 

#define POSITION_LOOP     1
#define SPEED_LOOP     2

#define pid_init() \
				do{ \
					chassis_center.pid.speed_loop.pid_Parameter.Vkp = 4; \
					chassis_center.pid.speed_loop.pid_Parameter.Vki = 0.1; \
					chassis_center.pid.speed_loop.pid_Parameter.Vkd = 0.1; \
					                   \
					chassis_motor1.pid.speed_loop.pid_Parameter.Vkp = 2.6;   \
					chassis_motor2.pid.speed_loop.pid_Parameter.Vkp = 2.6;   \
					chassis_motor3.pid.speed_loop.pid_Parameter.Vkp = 2.6;   \
					chassis_motor4.pid.speed_loop.pid_Parameter.Vkp = 2.6;   \
					chassis_motor1.pid.speed_loop.pid_Parameter.Vki = 0.35;   \
					chassis_motor2.pid.speed_loop.pid_Parameter.Vki = 0.35;   \
					chassis_motor3.pid.speed_loop.pid_Parameter.Vki = 0.35;   \
					chassis_motor4.pid.speed_loop.pid_Parameter.Vki = 0.35;   \
					chassis_motor1.pid.speed_loop.pid_Parameter.Vkd = 1;   \
					chassis_motor2.pid.speed_loop.pid_Parameter.Vkd = 1;   \
					chassis_motor3.pid.speed_loop.pid_Parameter.Vkd = 1;   \
					chassis_motor4.pid.speed_loop.pid_Parameter.Vkd = 1;   \
				}while(0) \

				
typedef enum
{
	CHASSIS = 1, //底盘模式
	FOLLOW =2,   //跟随模式
	NUL=0,
}switch_flag_t;//模式


/*速度pid*/
typedef struct{
	float err;
	float last_err;
	float err_integration;//累计误差
	float target_speed;
	float actual_speed;
	
	float P_OUT;
	float I_OUT;
	float D_OUT;
	float PID_OUT;//输出
	
	int pid_count;           
	float last_average_err;
    float average_err;    
}VPID_t;

typedef struct
{
	float Vkp;			
	float Vki;	
    float Vkd;	
}Parameter_t;//pid系数

typedef struct
{
	VPID_t vpid;
	Parameter_t pid_Parameter;
}PID_Loop_t;

typedef struct{
	uint8_t loop_flag;
	PID_Loop_t speed_loop;
}PID_t;


extern PID_t chassis_pid;
extern switch_flag_t switch_flag;
void motor_pid_init(PID_t *pid);
void pid_realize(PID_t *pid);

#endif
