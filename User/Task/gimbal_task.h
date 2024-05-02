#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "main.h"
#include "pid.h"

typedef struct
{
    uint16_t can_id;		//ID号
    int16_t  set_current;		//发送信息
    int16_t  torque_current;		//实际转矩电流
    uint8_t  temp;		//电机温度
	
	  //编码器
	  uint16_t rotor_angle;		//现在的角度
    int16_t  rotor_speed;		//现在的转速
	
	  //陀螺仪器
	  float gyro_yaw_angle;//云台yaw轴当前朝向
	  float gyro_pitch_angle; //云台pitch轴当前朝向
	
	  //目标值
	  float target_angle;//目标角度
	  int16_t target_speed;  //目标速度
	  
	  //初始值
	  float ZERO_motor; //初始时yaw轴电机编码值
	  float ZERO_gyro;  //初始时云台的朝向
	  
	  //PID参数
	  pid_struct_t gimbal_angle_pid;
	  pid_struct_t gimbal_speed_pid;
	
}gimbal_motor_info_t;


//=============================函数声明模块======================
static void gimbal_init();
void gimbal_mode_choice1(void);
void gimbal_mode_choice2(void);
void gimbal_current_give();

//过零处理
void detel_gyro(fp32* angle);
void detel_motor(fp32 *angle);

void gimbal_yaw_mode1();
void gimbal_yaw_mode2();

//锁云台模式
void rc_yaw_control();
void lock_gimbal_yaw();

//自瞄模式
void vision_arm();

void yaw_current_give();
	
void	pitch_current_give();

void Gimbal_gyro_speed_current();

void Gimbal_gyro_angle_current();


#endif





