#include "gimbal_task.h"
#include "Chassis_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "rc_map.h"
#include "encoder_map.h"
#include "drv_can.h"
#include "ins_task.h"
#include "rc_potocal.h"
#include "drv_usart.h"

float ZERO_yaw=0; 

gimbal_motor_info_t yaw_motor;
gimbal_motor_info_t pitch_motor;

extern motor_info_t  motor_info_chassis[4];       //�����Ϣ�ṹ��
extern INS_t INS;
extern RC_ctrl_t rc_ctrl;
extern UART_HandleTypeDef huart1;
extern char rx_buffer[100];

void Gimbal_task(void const * argument)
{
	osDelay(1);
	
	gimbal_init();
	
  for(;;)
  {
		//��yaw���pitch�����ģʽ��ѡ��
		gimbal_mode_choice1();
		
		gimbal_current_give();
		
    osDelay(1);
  }
  /* USER CODE END gimbal_task */
}


//��̨�����ʼ��
static void gimbal_init()
{
	//PID��������
	 float yaw_angle_pid[3]={2,0,0.3};
	 float yaw_speed_pid [3]={30,0.5,10};   

		pid_init(&yaw_motor.gimbal_angle_pid, yaw_angle_pid, 10000, 10000);  
	  pid_init(&yaw_motor.gimbal_speed_pid, yaw_speed_pid, 6000, 6000);
	 
	 //��ʼֵ��¼
	 yaw_motor.ZERO_motor = yaw_motor.rotor_angle;
	 yaw_motor.ZERO_gyro = INS.Yaw;
	 
	 pitch_motor.ZERO_gyro = INS.Pitch;
	 
	 //�Ӿ��ӿ�
	 __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ��IDLE�ж�
	HAL_UART_Receive_DMA(&huart1,(uint8_t*)rx_buffer,100); //��������,����minipc������
}


//ģʽѡ��
void gimbal_mode_choice1(void)
{
	
	if(rc_ctrl.rc.s[1]==3 ) //��ͨģʽ
	{
	  rc_yaw_control(); 
	}
	else if(rc_ctrl.rc.s[1]==1) //С����ģʽ
	{
		lock_gimbal_yaw();
	}
	else
	{
		yaw_motor.target_speed=0;
	}
	
	if(rc_ctrl.rc.s[0]==2)
	{
		vision_arm();
	}
}

void gimbal_current_give()
{
	yaw_current_give();
	
	pitch_current_give();
}

//===================================���㴦��===========================================
void detel_gyro(fp32* angle)
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

void detel_motor(fp32 *angle)
{
	if(*angle>4096)
	{
		*angle=*angle-8192;
	}
	else if(*angle<-4096)
	{
		*angle=*angle+8192;
	}
}



//==========================================��̨���Ʒ�ʽ===========================================================
//��ϽǶȻ������yaw������Ŀ���ٶ�
void rc_yaw_control()
{
	fp32 err_yaw=0;
	
	yaw_motor.ZERO_gyro=yaw_motor.ZERO_gyro+(get_xy_angle_8191(yaw_motor.ZERO_gyro)/4096*180);
	
	detel_gyro(&yaw_motor.ZERO_gyro);
	
	err_yaw=yaw_motor.ZERO_gyro - yaw_motor.gyro_yaw_angle;
	
	//��Ʈ�����Ŀ��Ƕȼ���
	if(err_yaw>1||err_yaw<-1)
	{
		
		//�˴����������̨����������ٶ�
		yaw_motor.target_speed=pid_calc(&yaw_motor.gimbal_angle_pid,yaw_motor.gyro_yaw_angle,yaw_motor.ZERO_gyro);
	}
	else 
	{  
		yaw_motor.target_speed=0;
	}
}


void lock_gimbal_yaw()
{
	fp32 err_yaw=0;
	
	yaw_motor.ZERO_gyro=yaw_motor.ZERO_gyro+rc_ctrl.rc.ch[3] / 660.0 * 360;
	
	detel_gyro(&yaw_motor.ZERO_gyro);
	
	err_yaw=yaw_motor.ZERO_gyro- yaw_motor.gyro_yaw_angle;
	
	//��Ʈ�����Ŀ��Ƕȼ���
	if(err_yaw>1||err_yaw<-1)
	{
		//�˴����������̨����������ٶ�
		yaw_motor.target_speed=pid_calc(&yaw_motor.gimbal_angle_pid,-err_yaw,0);
	}
	else 
	{  
		yaw_motor.target_speed=0;
	}
}

//����ģʽ
/*
void vision_arm()
{
	if(Pitch_minipc_fp)
	{
		pitch_motor.target_angle = motor_info[4].rotor_angle - Pitch_minipc_fp/360.0 * 8191; 
	}
	else
	{
		pitch_motor.target_angle =  pitch_motor.target_angle -((rc_ctrl.rc.ch[1]) / 660.0 * 2) + rc_ctrl.mouse.y / 16384.00 *1000; 
	}
	
	
	err_calc();
	
	target_speed[4] = pid_pitch_calc(&pitch_motor.gimbal_angle_pid,  pitch_motor.target_angle,  pitch_motor.gyro_pitch_angle);
	
	osDelay(1);

}
*/


//=========================================���̸�����̨ģʽ=====================================================
//������yaw�������������ڵ�������ϵ�ľ��ԽǶ�����
//���̸�����̨ģʽ����ͨ���Ե��̵�PID���ƣ�yaw���������������ǴӶ���ʹyaw_chassis����Ϊ��


//=======================================��̨���PID������㼰���Ƶ�������===============================================
fp32 gimbal_current=0;
//���Ѿ���ȡ�����ʵʱ�ٶȺ�����Ӧ������������Ŀ���ٶȵ�����£��������ٶȵ�PID����ͷ��͵���
void Gimbal_gyro_speed_current()
{
				
		gimbal_current=pid_calc(&yaw_motor.gimbal_speed_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
	  //gimbal_current��ֵУ��ϸ񣬲Ÿ�ֵ��yaw�����ṹ��
		if(gimbal_current>yaw_motor.target_speed)
		{
			gimbal_current=0;
		}
				
		yaw_motor.set_current = gimbal_current;	
    		
	  set_motor_current_gimbal(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}

void Gimbal_gyro_angle_current()
{
			
			  yaw_motor.target_speed= pid_pitch_calc(&yaw_motor.gimbal_angle_pid,yaw_motor.gyro_yaw_angle,yaw_motor.target_angle);
			
			  if((get_x_ch1()==0)&&(get_y_ch0()==0))
				{
					yaw_motor.target_speed=0;
				}

				
				gimbal_current=pid_calc(&yaw_motor.gimbal_speed_pid,yaw_motor.rotor_speed,yaw_motor.target_speed);
				
				if(gimbal_current>yaw_motor.target_speed)
				{
					gimbal_current=0;
				}
				
				yaw_motor.set_current = gimbal_current;
				
				set_motor_current_chassis(0,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current ,yaw_motor.set_current );
}



