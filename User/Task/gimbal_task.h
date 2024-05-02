#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    uint16_t can_id;		//ID��
    int16_t  set_current;		//������Ϣ
    int16_t  torque_current;		//ʵ��ת�ص���
    uint8_t  temp;		//����¶�
	
	  //������
	  uint16_t rotor_angle;		//���ڵĽǶ�
    int16_t  rotor_speed;		//���ڵ�ת��
	
	  //��������
	  float gyro_yaw_angle;//��̨yaw�ᵱǰ����
	  float gyro_pitch_angle; //��̨pitch�ᵱǰ����
	
	  //Ŀ��ֵ
	  float target_angle;//Ŀ��Ƕ�
	  int16_t target_speed;  //Ŀ���ٶ�
	  
	  //��ʼֵ
	  float ZERO_motor; //��ʼʱyaw��������ֵ
	  float ZERO_gyro;  //��ʼʱ��̨�ĳ���
	  
	  //PID����
	  pid_struct_t gimbal_angle_pid;
	  pid_struct_t gimbal_speed_pid;
	
}gimbal_motor_info_t;


//=============================��������ģ��======================
static void gimbal_init();
void gimbal_mode_choice1(void);
void gimbal_mode_choice2(void);
void gimbal_current_give();

//���㴦��
void detel_gyro(fp32* angle);
void detel_motor(fp32 *angle);

void gimbal_yaw_mode1();
void gimbal_yaw_mode2();

//����̨ģʽ
void rc_yaw_control();
void lock_gimbal_yaw();

//����ģʽ
void vision_arm();

void yaw_current_give();
	
void	pitch_current_give();

void Gimbal_gyro_speed_current();

void Gimbal_gyro_angle_current();


#endif





