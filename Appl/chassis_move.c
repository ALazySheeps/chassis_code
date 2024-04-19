/**
 *****************************东莞理工学院ACE实验室 *****************************
 * @file chassis_move.c
 * @brief 底盘移动控制，针对得到的模式来进行对应的输出（和底盘功率限制）
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

#include "chassis_move.h"

static void chassis_speed_control(float speed_x, float speed_y, float speed_r);
static void chassis_move_mode(void);
static void can_send_chassis_current(void);
//static void power_limitation_jugement(void);
void SET_SPEED(float vx,float vy,float wz);

//float total_current_limit,total_current,power,buffer,max_power;
//float BUFFER_TOTAL_CURRENT_LIMIT=5000.0f,POWER_TOTAL_CURRENT_LIMIT=9000.0f;//功率限制函数参数(若易超功率则适当减小)

CHASSIS_CONTROL_ORDER_t chassis_control_order;
MOTOR_t chassis_motor1,chassis_motor2,chassis_motor3,chassis_motor4,chassis_center;

void chassis_move(void)//底盘移动过程
{
	//对应模式的底盘状态赋值
	chassis_move_mode();  //设置pid.target_speed
	//pid计算
	vpid_chassis_realize();//计算pid_OUT
	//功率判断与限制
	//power_limitation_jugement();
	//输出底盘电流值
	can_send_chassis_current();//发送给CAN
}

int flag3=0;
static void chassis_speed_control(float speed_x, float speed_y, float speed_r)
{
	flag3=1;
	float max;
		//麦轮解算
		BaseVel_To_WheelVel(speed_x, speed_y, speed_r);

		max=find_max();
		if(max>MAX_MOTOR_SPEED)
		{
			chassis_motor1.target_speed= chassis_motor1.target_speed*(MAX_MOTOR_SPEED / max);
			chassis_motor2.target_speed= chassis_motor2.target_speed*(MAX_MOTOR_SPEED / max);
			chassis_motor3.target_speed= chassis_motor3.target_speed*(MAX_MOTOR_SPEED / max);
			chassis_motor4.target_speed= chassis_motor4.target_speed*(MAX_MOTOR_SPEED / max);
		}
	set_chassis_speed(chassis_motor1.target_speed, chassis_motor2.target_speed, chassis_motor3.target_speed, chassis_motor4.target_speed);
}	

void SET_SPEED(float VX,float VY,float WZ)
{
    chassis_control_order.vx_set = VX*(15535.0/660.0)*2;
    chassis_control_order.vy_set = VY*(15535.0/660.0)*2;
    chassis_control_order.wz_set = WZ*50/660;
}

int flag=0;
static void chassis_move_mode(void)//对应模式下的底盘动作
{
	float vx,vy,wz;
	if(chassis_control_order.chassis_mode==CHASSIS_REMOTE_CLOSE)
	{
        chassis_speed_control(0,0,0);
	}
    else if(chassis_control_order.chassis_mode==CHASSIS_NORMAL)//这里是底盘移动的
	{
		vx = chassis_control_order.vx_set;
		vy = chassis_control_order.vy_set;
		wz = chassis_control_order.wz_set;
		chassis_speed_control(vx,vy,wz);
	}
/*******************************！！！！！！！！***********************************************/
}

int flag1=0;
static void can_send_chassis_current(void)//输出底盘电流值
{
	flag1=1;
	static uint8_t cdata[8];
	//cdata[0]= ( ((int)chassis_motor1.pid.speed_loop.vpid.PID_OUT)>>8 )&0xFF;
	//cdata[1]= ((int)chassis_motor1.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[2]= ( ((int)chassis_motor2.pid.speed_loop.vpid.PID_OUT)>>8 )&0xFF;
	cdata[3]= ((int)chassis_motor2.pid.speed_loop.vpid.PID_OUT)&0xFF;
	cdata[4]= ( ((int)chassis_motor3.pid.speed_loop.vpid.PID_OUT)>>8 )&0xFF;
	cdata[5]= ((int)chassis_motor3.pid.speed_loop.vpid.PID_OUT)&0xFF;
	//cdata[6]= ( ((int)chassis_motor4.pid.speed_loop.vpid.PID_OUT)>>8 )&0xFF;
	//cdata[7]= ((int)chassis_motor4.pid.speed_loop.vpid.PID_OUT)&0xFF;
	
	Can_Tx_Message(&hcan1,cdata);
}

/*
static void power_limitation_jugement(void)//底盘功率限制
{
	static uint8_t flag20=0;
	float temp;
	total_current=0;

  //power > WARNING_POWER
  //功率大于WARNING_POWER   40
  get_chassis_power_and_buffer_and_max(&power, &buffer,&max_power);
    buffer=60;
  // power > 80w and buffer < 60j, because buffer < 60 means power has been more than 80w
  //功率超过80w 和缓冲能量小于60j,因为缓冲能量小于60意味着功率超过80w
  if(buffer < WARNING_POWER_BUFF)//50
  {
    float power_scale;
    if(buffer > 5.0f)
    {
			//scale down WARNING_POWER_BUFF
			//缩小WARNING_POWER_BUFF
			power_scale = buffer / WARNING_POWER_BUFF;
    }
    else
    {
     //only left 10% of WARNING_POWER_BUFF
      power_scale = 5.0f / WARNING_POWER_BUFF;
    }
     //scale down
     //缩小
   total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT * power_scale;
  }
	else
	{
		//power > WARNING_POWER
		//功率大于WARNING_POWER
		if(power > WARNING_POWER)
		{
			float power_scale;
			//power < 80w
			//功率小于80w
			if(power < POWER_LIMIT)
			{
				//scale down
				//缩小
				power_scale = (POWER_LIMIT - power) / (POWER_LIMIT - WARNING_POWER);	
			}
			//power > 80w
			//功率大于80w
			else
			{
				power_scale = 0.0f;
			}
				 
			total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT * power_scale;
		}
			//power < WARNING_POWER
			//功率小于WARNING_POWER
		else
		{
		  total_current_limit = BUFFER_TOTAL_CURRENT_LIMIT + POWER_TOTAL_CURRENT_LIMIT;
		}
	}

	total_current += abs(chassis_motor1.pid.speed_loop.vpid.PID_OUT);
	total_current += abs(chassis_motor2.pid.speed_loop.vpid.PID_OUT);
	total_current += abs(chassis_motor3.pid.speed_loop.vpid.PID_OUT);
	total_current += abs(chassis_motor4.pid.speed_loop.vpid.PID_OUT);


	if(total_current > total_current_limit)//判断电流（-16384~16384）是否大于限制total_current_limit
	{
		float current_scale = total_current_limit / total_current;
		chassis_motor1.pid.speed_loop.vpid.PID_OUT*=current_scale;
		chassis_motor2.pid.speed_loop.vpid.PID_OUT*=current_scale;
		chassis_motor3.pid.speed_loop.vpid.PID_OUT*=current_scale;
		chassis_motor4.pid.speed_loop.vpid.PID_OUT*=current_scale;
	}
}*/
