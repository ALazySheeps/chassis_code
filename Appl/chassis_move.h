#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "kinematics.h"
#include "can.h"
#include <math.h>

//底盘模式
#define CHASSIS_REMOTE_CLOSE   	 1    //关闭遥控器       左高位
#define CHASSIS_NORMAL           3    //正常模式         左中位
#define CHASSIS_SPIN             2    //小陀螺模式       左低位

//功率限制
#define POWER_LIMIT         80.0f
#define WARNING_POWER       40.0f   
#define WARNING_POWER_BUFF  50.0f  

#define vpid_chassis_realize() \
				do{ \
						switch_flag=CHASSIS;      \
						pid_realize(&(chassis_motor1.pid));   \
						pid_realize(&(chassis_motor2.pid));   \
						pid_realize(&(chassis_motor3.pid));   \
						pid_realize(&(chassis_motor4.pid));   \
                        switch_flag=NUL;  \
				}while(0)               \

typedef struct{
	
//	int16_t target_angle;
	
//	int16_t actual_angle;			//当前真实角度值
//	int16_t last_angle;				//上一次返回的角度值
//	int16_t switch_mode_angle;  //记录模式转换角度值
//	int round_cnt;				//相对开机时转过的圈数
//	int total_angle;			//总共转过的计数
	
	float actual_speed;			//电机真实速度,rpm
	float target_speed;			//电机目标速度,rpm  转/min
	float last_speed;       //电机上一次回传的速度值
    
	PID_t pid;
    }MOTOR_t;                  //电机结构体

typedef struct
{
	float vx_set;
	float vy_set;
	float wz_set;              //x、y、z的目标值
	uint8_t chassis_mode;        //底盘模式
}CHASSIS_CONTROL_ORDER_t;        //底盘控制目标结构体

extern MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;
extern CHASSIS_CONTROL_ORDER_t chassis_control_order;
void chassis_move(void);
void SET_SPEED(float VX,float VY,float WZ);
#endif
