#include "Chassis_task.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "exchange.h"
#include "drv_can.h"
#include "rc_map.h"
#include "gimbal_task.h"
#include "pid.h"

#define LENGTH_A 
#define LENGTH_B


extern gimbal_motor_info_t yaw_motor;
extern RC_ctrl_t rc_ctrl;
extern float powerdata[4];
extern uint16_t shift_flag;
int8_t chassis_mode = 1;//判断底盘状态，用于UI编写


motor_info_t  motor_info_chassis[4];       //电机信息结构体
volatile int16_t motor_speed_target[4];
volatile uint16_t motor_angle_target[4];

chassis_direct_t chassis_direct;
pid_struct_t chassis_pid_direct;

void Chassis_task(void const * argument)
{    
	  osDelay(1);
	
	  Chassis_Init();
    
    for(;;)				//底盘运动任务
    {     
			 chassis_loop();
			 
			 chassis_mode_choice();
         
			 chassis_motol_speed_calculate();
		
       //PID计算应该发送给4个电机分别的电流
       chassis_current_give();
            
       osDelay(1);

    }

}


volatile int16_t Vx=0,Vy=0,Wz=0;

void Chassis_Init()
{
		//PID的参数数组
	fp32 motor_speed_pid [3]={30,0.5,10};   //用的原来的pid
	fp32 chassis_angle_pid[3]={20,0,0.3};
	
	for (uint8_t i = 0; i < 4; i++)
	  {
        pid_init(&motor_info_chassis[i].motor_pid, motor_speed_pid, 6000, 6000); //init pid parameter, kp=40, ki=3, kd=0, output limit = 16384
	  } 

		pid_init(&chassis_pid_direct,chassis_angle_pid,10000,10000);
		
}

void chassis_loop()
{
	Vx=0;
	Vy=0;
	Wz=0;
}


void chassis_mode_choice()
{
	if(rc_ctrl.rc.s[1]==3 ) //非小陀螺模式
	{
		if(rc_ctrl.rc.s[0]==3)
		{
			 chassis_mode1();
		}
		else if(rc_ctrl.rc.s[0]==1) //底盘跟随云台
		{
			 chassis_mode2();
		}
	 
	}
	else if(rc_ctrl.rc.s[1]==1) //小陀螺模式下
	{
		chassis_mode3();
	}
}

//普通模式
void chassis_mode1()
{
	Vx = rc_ctrl.rc.ch[0];
	Vy = rc_ctrl.rc.ch[1];
	Wz = rc_ctrl.rc.ch[2];
}

//底盘跟随云台
/*
void chassis_mode2()
{
  float err_chassis = 0;
	err_chassis = yaw_motor.rotor_angle - yaw_motor.ZERO_motor;
	detel_motor(& err_chassis);
	
  Wz = pid_calc(&chassis_pid_direct,-err_chassis,0);
	
	Vx = rc_ctrl.rc.ch[0];
	Vy = rc_ctrl.rc.ch[1];
}
*/
void chassis_mode2()
{
	float angle_diff =0;
	int16_t  gimbal_Vx,gimbal_Vy;
	angle_diff = yaw_motor.rotor_angle - yaw_motor.ZERO_motor;
	detel_motor(&angle_diff);
	
	gimbal_Vx = rc_ctrl.rc.ch[0];
	gimbal_Vy = rc_ctrl.rc.ch[1];
	
	//底盘坐标系下的速度与云台坐标系相互转换
	Vx = gimbal_Vx * cos(angle_diff) - gimbal_Vy * sin(angle_diff);
	Vy = gimbal_Vx * sin(angle_diff) - gimbal_Vy * cos(angle_diff);
	
	/*对底盘跟随进行双环控制,外环为位置环,
	  目标值为相对角度差，实际值为0*/
	float err_chassis = 0;
	err_chassis = yaw_motor.rotor_angle - yaw_motor.ZERO_motor;
	detel_motor(& err_chassis);
	
	Wz= pid_calc(&chassis_pid_direct,-err_chassis,0);
}

//小陀螺+平动
void chassis_mode3()
{
	Wz = 300;
	
	float err_chassis = 0;
	err_chassis = yaw_motor.rotor_angle - yaw_motor.ZERO_motor;
	detel_motor(& err_chassis);
	err_chassis = (err_chassis/8192)*360;
	
	Vx = cos(err_chassis)*rc_ctrl.rc.ch[0]-sin(err_chassis)*rc_ctrl.rc.ch[1];
	Vx*=300;
	
	Vy = sin(err_chassis)*rc_ctrl.rc.ch[0]-cos(err_chassis)*rc_ctrl.rc.ch[1];
	Vx*=300;
}

//运动解算
void chassis_motol_speed_calculate(void)
{
	  motor_speed_target[CHAS_LF] =  0;
    motor_speed_target[CHAS_RF] =  0;
    motor_speed_target[CHAS_RB] =  0; 
    motor_speed_target[CHAS_LB] =  0;
	
	  motor_speed_target[CHAS_LF] =Vx -Vy+Wz;
    motor_speed_target[CHAS_RF] =Vx +Vy +Wz;
    motor_speed_target[CHAS_RB] =-Vx+Vy+Wz; 
    motor_speed_target[CHAS_LB] =-Vx -Vy+Wz;
}

//速度限制函数
  void Motor_Speed_limiting(volatile int16_t *motor_speed,int16_t limit_speed)  
{
    uint8_t i=0;
    int16_t max = 0;
    int16_t temp =0;
    int16_t max_speed = limit_speed;
    fp32 rate=0;
    for(i = 0; i<4; i++)
    {
      temp = (motor_speed[i]>0 )? (motor_speed[i]) : (-motor_speed[i]);//求绝对值
		
      if(temp>max)
        {
          max = temp;
        }
     }	
	
    if(max>max_speed)
    {
          rate = max_speed*1.0/max;   //*1.0转浮点类型，不然可能会直接得0   *1.0 to floating point type, otherwise it may directly get 0
          for (i = 0; i < 4; i++)
        {
            motor_speed[i] *= rate;
        }

    }
}

//电机电流控制
void chassis_current_give() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {                                                                       
        motor_info_chassis[i].set_current = pid_calc(&motor_info_chassis[i].motor_pid, motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
    }
    set_motor_current_chassis(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);
}
//=========================================================================================================================

//============================底盘任务模式及模式切换===============================================
//============================1.底盘朝向固定
//=================1.1底盘搭载陀螺仪的情况
fp32 ZERO_chassis=0;
void lock_chassis_direct()
{
	fp32 chassis_err=0;
	
	chassis_err=chassis_direct.gyro_angle-ZERO_chassis;
	
	detel_motor(&chassis_err);
	
	//此处的目标速度是整车的目标旋转角速度Wz
	chassis_direct.target_speed=pid_calc(&chassis_pid_direct,-chassis_err,0);
	
	Wz=chassis_direct.target_speed;
}

//越界处理
/*
void detel_chassis(fp32* angle)
{
	if(*angle>180)
	{
		*angle=*angle-360;
	}
	else if(*angle<-180)
	{
		*angle=*angle+360;
	}
}
*/

//=================1.2只有云台搭载陀螺仪-底盘跟随（通过yaw轴电机编码器和云台陀螺仪控制）
//云台与底盘相联系的部分：yaw轴6020
//底盘跟随云台，即使编码器的编码值保持不变
fp32 yaw_err=0;

void Chassis_following()
{
	fp32 Wz_max=6000;
	fp32 angle_value=3;
	fp32 angle_weight=30;
	
	yaw_err=Get_err();
	
	if(yaw_err>3||yaw_err<-3)
	{
		Wz=yaw_err*angle_weight;//设Wz的转速为 角度差*权重，让底盘转回去
	}
	
	//速度限幅
	if(Wz>Wz_max)
	{
		Wz=Wz_max;
	}
	else if(Wz<-Wz_max)
	{
	  Wz=-Wz_max;
	}
}

//以yaw轴陀螺仪的值维目标值，底盘陀螺仪为当前值
float Get_err()
{
	 yaw_err=encoder_map_360(yaw_motor.ZERO_gyro,yaw_motor.gyro_yaw_angle)
	          -encoder_map_360(chassis_direct.ZERO_gyro,chassis_direct.gyro_angle);
	if(yaw_err>180)
	{
		yaw_err=yaw_err-360;
	}
	else if(yaw_err<-180)
	{
		yaw_err=yaw_err+360;
	}
	
	return yaw_err;
}



//============================2.小陀螺模式
//=================2.1底盘高速旋转时，云台朝向保持不变
//=================2.2底盘高速旋转的同时，云台朝向可以用遥控器调控
//不管底盘是不是在旋转，只要保持陀螺仪的朝向不变就可以了
void small_xiaoyuoluo()
{
	fp32 ZERO_gimbal_gyro=0;
	ZERO_gimbal_gyro= ZERO_gimbal_gyro+get_xy_angle_8191(ZERO_gimbal_gyro);
	
	fp32 err_gyro=0;
	gimbal_motor_info_t yaw_gyro;
	
	err_gyro=yaw_gyro.gyro_yaw_angle-ZERO_gimbal_gyro;
	
	if(err_gyro>180)
	{
		err_gyro-=360;
	}
	else if(err_gyro<-180)
	{
		err_gyro+=360;
	} 
	
	//yaw_gyro.target_speed=pid_calc(-err_gyro,0)
	//yaw_gyro.set_current=pid_calc(yaw_gyro.rotor_speed,yaw_gyro.target_speed)
		
}


//============================用遥控器控制6020转过固定角度=========================================
fp32 current=0;
//这个函数在步兵中应该用于云台gimbal的控制
void chassis_current_give_RC_6020() 
{
	
    uint8_t i=0;
        
    for(i=0 ; i<4; i++)
    {
			
			  motor_speed_target[i]=pid_pitch_calc(&motor_info_chassis[i].motor_pid,
			                                        encoder_map_8191(yaw_motor.ZERO_motor,yaw_motor.rotor_angle),
			                                         get_xy_angle_8191(yaw_motor.ZERO_motor));
						
			  if((get_x_ch1()==0)&&(get_y_ch0()==0))
				{
					motor_speed_target[i]=0;
				}
			
				
				current=pid_calc(&motor_info_chassis[i].motor_pid, motor_info_chassis[i].rotor_speed,motor_speed_target[i]);
				
				if(current>motor_speed_target[i])
				{
					current=0;
				}
				
				motor_info_chassis[i].set_current = current;
    }
    	
		set_motor_current_chassis(0, motor_info_chassis[0].set_current, motor_info_chassis[1].set_current, motor_info_chassis[2].set_current, motor_info_chassis[3].set_current);

}


