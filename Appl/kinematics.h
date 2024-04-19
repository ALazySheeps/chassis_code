#ifndef __KINEMATICS_H
#define __KINEMATICS_H

#include "chassis_move.h"
#include <stdlib.h>
#define wheel_diameter  154.00000f			//轮子直径
#define WHEEL_WIDTH    590.00000f            //底盘宽
#define WHEEL_LENGTH   580.00000f            //底盘长
#define GIMBAL_OFFSET 0                    //悬挂偏移量（假设没有偏移）

#define PI 			  3.140000f
#define PI2 	       6.280000f
#define RADIAN_COEF   57.3f                     //弧度转换系数     1rad=57.3度
#define M3508_REDUCTION_RATIO 19.000000f		//齿轮箱减速比
#define GM6020_ENCODER_ANGLE  8192.0f           //角度范围

#define MAX_MOTOR_SPEED   15336				//电机最大转速


#define set_chassis_speed(motor1_speed,motor2_speed,motor3_speed,motor4_speed) \
        do{                                                                    \
			chassis_motor1.pid.speed_loop.vpid.target_speed = motor1_speed;	             \
	        chassis_motor2.pid.speed_loop.vpid.target_speed = motor2_speed;              \
	        chassis_motor3.pid.speed_loop.vpid.target_speed = motor3_speed;              \
	        chassis_motor4.pid.speed_loop.vpid.target_speed = motor4_speed;              \
				}while(0)                                                               \



void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
float find_max(void);

#endif
