/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file kinematics.c
 * @brief 运动学公式，实现四轮与底盘间速度的转换
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


#include "kinematics.h"

void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z);
float find_max(void);

//四轮位置关系：
//4  1
//3  2
//1 、2号电机是反的
void BaseVel_To_WheelVel(float linear_x, float linear_y, float angular_z)//vx_set vy_set wz_set
{
    //计算前轮、后轮的转动比
    float rotate_ratio_f = (WHEEL_WIDTH + WHEEL_LENGTH)/2 - GIMBAL_OFFSET;
    float rotate_ratio_d = (WHEEL_WIDTH + WHEEL_LENGTH)/2 + GIMBAL_OFFSET;
    
    //计算轮子rpm比例系数
    //cm/s转换成每分钟的移动距离再除以周长和减速比得到rad/min
    //解算出四个轮子的目标速度 rpm
    chassis_motor1.target_speed = (-linear_x + linear_y - angular_z*rotate_ratio_f)/wheel_diameter *60/PI2 ; 
    chassis_motor2.target_speed = (-linear_x - linear_y - angular_z*rotate_ratio_d)/wheel_diameter *60/PI2 ;
    chassis_motor3.target_speed = (+linear_x - linear_y - angular_z*rotate_ratio_d)/wheel_diameter *60/PI2 ;
    chassis_motor4.target_speed = (+linear_x + linear_y - angular_z*rotate_ratio_f)/wheel_diameter *60/PI2 ;
    
}


float find_max(void)//四个轮子的最大值   
{
  float temp=0;
  
  temp=abs((int)chassis_motor1.target_speed);
  if(abs((int)chassis_motor2.target_speed)>temp)
    temp=abs((int)chassis_motor2.target_speed);
  if(abs((int)chassis_motor3.target_speed)>temp)
    temp=abs((int)chassis_motor3.target_speed);
  if(abs((int)chassis_motor4.target_speed)>temp)
    temp=abs((int)chassis_motor4.target_speed);
  return temp;
}

