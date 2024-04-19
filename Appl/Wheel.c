#include "Wheel.h"




//设置麦轮速度值
//x:横向速度；y：纵向速度；z：旋转角度
//对应四个轮子的位置：
//4    1
//3    2
void McNamara_Set_Val(float x,float y,float w,float *Speed)
{
    float rotate_ratio_f = ((WHEELBASE+WHEELTRACK)/2-GIMBAL_OFFSET)*RADIAN_COEF;  //前轮转动比
    float rotate_ratio_d = ((WHEELBASE+WHEELTRACK)/2+GIMBAL_OFFSET)*RADIAN_COEF;  //后轮转动比
    //float whell_rpm_ratio = (RPM_CODE) / (CHASSIS_DECELE_RATIO * PERIMETER);  //转rpm
    float whell_rpm_ratio = 0.065;
    *(Speed + 0) = (+x - y + w*rotate_ratio_f)*whell_rpm_ratio;
    *(Speed + 1) = (-x - y + w*rotate_ratio_f)*whell_rpm_ratio;
    *(Speed + 2) = (-x + y + w*rotate_ratio_d)*whell_rpm_ratio;
    *(Speed + 3) = (+x + y + w*rotate_ratio_d)*whell_rpm_ratio;       //0-3号轮的速度值
}

//pid结构体

 void pid_Init(_pid *pid,float p,float i,float d)
 {
     /* 初始化参数 */
     for(int i = 0;i < 4;i ++)
     {
         pid[i].target_val=0.0;
         pid[i].actual_val=0.0;
         pid[i].err=0.0;
         pid[i].err_last=0.0;
         pid[i].sum=0.0;
         pid[i].Kp = p;
         pid[i].Ki = i;
         pid[i].Kd = d;
     }
 }
 

 /**
  * @bref pid算法实现
  * @pram 实际速度
  * @pram 目标速度
  **/
 float PID_realize(float temp_val,_pid pid)
{
     /*目标值只在这里参与计算，计算目标值与实际值的误差*/
    pid.err=pid.target_val-temp_val;
     /*误差累积*/
    pid.sum+=pid.err;
    //此处可以进行误差限幅
    if(pid.sum > 5000) pid.sum = 5000;
    
     /*PID算法实现*/
    pid.actual_val=pid.Kp*pid.err+pid.Ki*pid.sum+pid.Kd*(pid.err-pid.err_last);
     /*误差传递*/
    pid.err_last=pid.err;
     /*返回的值是经过pid运算以后的值*/
    return pid.actual_val;
}
 
 
