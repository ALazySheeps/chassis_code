#ifndef __WHEEL_H
#define __WHEEL_H

#include "main.h"

#define WHEELBASE 17.0             //轴距
#define WHEELTRACK 21.0             //轮距
#define GIMBAL_OFFSET 0          //悬挂偏移量
#define RADIAN_COEF 57.3      //弧度转角度系数

#define PERIMETER 48.356;  //周长

#define CHASSIS_DECELE_RATIO 19.0  //减速比
#define RPM_CODE 60.0


typedef struct
 {
     float target_val;             //目标值
     float actual_val;             //控制值
     float err;                    //定义偏差值
     float err_last;               //定义上一个偏差值
     float Kp,Ki,Kd;               //定义比例、积分、微分系数
     float sum;                    //定义历史偏差
 }_pid; 
 

void McNamara_Set_Val(float x,float y,float w,float *Speed);
 void pid_Init(_pid *pid,float p,float i,float d);

 //pid算法实现
float PID_realize(float temp_val,_pid pid);

#endif
