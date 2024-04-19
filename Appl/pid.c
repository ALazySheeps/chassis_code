/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file pid.c
 * @brief pid控制
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
#include "pid.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }



switch_flag_t switch_flag;
static void vpid_realize(VPID_t *vpid,Parameter_t *loop_p);

static void speed_loop_realize(PID_Loop_t *loop_select);

static void speed_loop_init(PID_Loop_t *loop_select);
static void VPID_Init(VPID_t *vpid);

		
void motor_pid_init(PID_t *pid)
{
	if(pid->loop_flag==SPEED_LOOP)
	speed_loop_init(&(pid->speed_loop));
}

static void speed_loop_init(PID_Loop_t *loop_select)
{
	VPID_Init(&(loop_select->vpid));
}


static void VPID_Init(VPID_t *vpid)
{
	vpid->target_speed=0;
	vpid->actual_speed=0;
	vpid->err=0;
	vpid->last_err=0;
	vpid->err_integration=0;
	vpid->P_OUT=0;
	vpid->I_OUT=0;
	vpid->D_OUT=0;
	vpid->PID_OUT=0;
	vpid->pid_count=0;
	vpid->average_err=0;
	vpid->last_average_err=0; 
}



void pid_realize(PID_t *pid)
{
	if(pid->loop_flag==SPEED_LOOP)  //速度回环
	speed_loop_realize(&(pid->speed_loop));
}

static void speed_loop_realize(PID_Loop_t *loop_select)
{
	vpid_realize(&(loop_select->vpid),&(loop_select->pid_Parameter));
}

//根据对应的模式来进行pid计算（目前仅底盘模式）
static void vpid_realize(VPID_t *vpid,Parameter_t *loop_p)
{
	vpid->err = vpid->target_speed - vpid->actual_speed;
	switch(switch_flag)
	{
	case(CHASSIS):
	 {
		if(vpid->err <= CHASSIS_IntegralSeparation || vpid->err >= -CHASSIS_IntegralSeparation)		//积分饱和500
			vpid->err_integration += vpid->err;
        else vpid->err_integration += CHASSIS_IntegralSeparation;
        
	    if(vpid->err_integration > CHASSIS_Integral_max)		//抗积分饱和5000
            vpid->err_integration = CHASSIS_Integral_max;
	    else if(vpid->err_integration < -CHASSIS_Integral_max)
            vpid->err_integration = -CHASSIS_Integral_max;
		
		vpid->P_OUT = loop_p->Vkp * vpid->err;			    	//P项
	    vpid->I_OUT = loop_p->Vki * vpid->err_integration;		//I项
		
		//输出限幅
		if((vpid->P_OUT + vpid->I_OUT )> CHASSIS_vPID_max) //5000
		vpid->PID_OUT = CHASSIS_vPID_max;
	  else if((vpid->P_OUT + vpid->I_OUT ) < -CHASSIS_vPID_max) 
		vpid->PID_OUT = -CHASSIS_vPID_max;
	  else
		vpid->PID_OUT = vpid->P_OUT + vpid->I_OUT;
   }
	 break;
		
	default:break;
 }
}
