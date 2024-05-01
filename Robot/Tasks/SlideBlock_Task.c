/**
  **********************************2022 CKYF***********************************
  * @file    SlideBlock_Task.c
  * @brief   滑块控制任务
  ******************************************************************************
	* @team    长空御风
	* @author  杨洛
	* @debug   曾新宇
  ******************************************************************************
  * @attention
  * 
  * 此任务用于控制滑块的运动，包含滑块控制主任务，以及滑块运动的各种模式。
  *
  **********************************2022 CKYF***********************************
  */
#include "SlideBlock_Task.h"
#include "DM9015.h"
#include "pid.h"
#include "ChassisSwitch_Task.h"
#include "ChassisControl_Task.h"
#include "ChassisDetect_Task.h"
#include "filter.h"
#include "arm_math.h"
#include "AHRS_MiddleWare.h"


#define DM9015_MOTOR_PID_KP 0.5f
#define DM9015_MOTOR_PID_KI 0.1f
#define DM9015_MOTOR_PID_KD 0.1f
#define DM9015_MOTOR_PID_MAX_OUT 800.0f
#define DM9015_MOTOR_PID_MAX_IOUT 80.0f

SlideBlock_control_t SlideBlock_ctrl;
pid_type_def PID_DM9015[2];
float DM9015_pid[3] = {DM9015_MOTOR_PID_KP, DM9015_MOTOR_PID_KI, DM9015_MOTOR_PID_KD};
lpf_data_t lpf_delta_speed;
fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
float K_temp=10,x_temp=1.3;

/**
  * @brief  滑块控制任务
  */
void SlideBlock_Task(void const * argument)
{
	vTaskDelete(SlideBlock_TaskHandle);  //不使用滑块
	
	vTaskDelay(2000);
	DM9015_measure[0].OffsetAngle=2930;
	DM9015_measure[1].OffsetAngle=79;
	PID_init(&PID_DM9015[0], PID_POSITION, DM9015_pid, DM9015_MOTOR_PID_MAX_OUT, DM9015_MOTOR_PID_MAX_IOUT);
	PID_init(&PID_DM9015[1], PID_POSITION, DM9015_pid, DM9015_MOTOR_PID_MAX_OUT, DM9015_MOTOR_PID_MAX_IOUT);
	LP_FilterInit(&lpf_delta_speed,fliter_num);
	DM9015_send(0,1);
	while(1)
	{
		if(SlideBlock_ctrl.ctrl_mode == BLOCK_NORMAL)
			SlideBlock_normal();
		
		if(SlideBlock_ctrl.ctrl_mode == BLOCK_READY)
			SlideBlock_ready();
		
		if(SlideBlock_ctrl.ctrl_mode == BLOCK_WEAK)
			SlideBlock_weak();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////		
		uint8_t DM9015_RX_ID = ulTaskNotifyTake(pdTRUE,100);
		if(DM9015_RX_ID == 2)
		{
			DM9015_send(SlideBlock_ctrl.GivenValue[0],1);
			vTaskDelay(1);
			continue;
		}
		if(DM9015_RX_ID == 1)
		{
			DM9015_send(SlideBlock_ctrl.GivenValue[1],2);
			vTaskDelay(1);
			continue;
		}
		vTaskDelay(200);
		DM9015_send(0,1);
		vTaskDelay(200);
		DM9015_send(0,2);
	}
}


float PID_normal[3] = {1.5f,0.2f,5.0f};
/**
  * @brief  滑块运动模式
  */
void SlideBlock_normal()
{
	PID_DM9015[0].Kp=PID_DM9015[1].Kp = PID_normal[0];
	PID_DM9015[0].Ki=PID_DM9015[1].Ki = PID_normal[1];
	PID_DM9015[0].Kd=PID_DM9015[1].Kd = PID_normal[2];
	
	float delta_spd=fabs(arm_cos_f32(PID_LQR_k5.error[0]))*limit(LP_FilterCalc(&lpf_delta_speed,(chassis_ctrl.move_speed)),-4,4);
	float SetAngle = 1300 * Logistic(fabs(delta_spd),K_temp,x_temp,1);
	if(delta_spd<0)SetAngle *= -1;
//	SetAngle *= fabs(arm_cos_f32(PID_LQR_k5.error[0]));
	DM9015_measure[0].SetAngle = SetAngle;
	DM9015_measure[1].SetAngle = -SetAngle;
	PID_calc(&PID_DM9015[0],DM9015_measure[0].TotalAngle,DM9015_measure[0].SetAngle);
	PID_calc(&PID_DM9015[1],DM9015_measure[1].TotalAngle,DM9015_measure[1].SetAngle);
	SlideBlock_ctrl.GivenValue[0] = PID_DM9015[0].out;
	SlideBlock_ctrl.GivenValue[1] = PID_DM9015[1].out;
}


float PID_weak[3] = {0.5f,0.1f,1.0f};
/**
  * @brief  滑块无力模式
  */
void SlideBlock_weak()
{
	PID_DM9015[0].Kp=PID_DM9015[1].Kp = PID_weak[0];
	PID_DM9015[0].Ki=PID_DM9015[1].Ki = PID_weak[1];
	PID_DM9015[0].Kd=PID_DM9015[1].Kd = PID_weak[2];
	
	DM9015_measure[0].SetAngle = 0;
	DM9015_measure[1].SetAngle = 0;
	PID_calc(&PID_DM9015[0],DM9015_measure[0].TotalAngle,DM9015_measure[0].SetAngle);
	PID_calc(&PID_DM9015[1],DM9015_measure[1].TotalAngle,DM9015_measure[1].SetAngle);
	SlideBlock_ctrl.GivenValue[0] = PID_DM9015[0].out;
	SlideBlock_ctrl.GivenValue[1] = PID_DM9015[1].out;
}

/**
  * @brief  滑块准备模式
  */
void SlideBlock_ready()
{
	PID_DM9015[0].Kp=PID_DM9015[1].Kp = PID_normal[0];
	PID_DM9015[0].Ki=PID_DM9015[1].Ki = PID_normal[1];
	PID_DM9015[0].Kd=PID_DM9015[1].Kd = PID_normal[2];
	
	float SetAngle;
	if(SlideBlock_ctrl.angle_x>=0)SetAngle = -1300;
	else SetAngle = 1300;
	
	DM9015_measure[0].SetAngle = SetAngle;
	DM9015_measure[1].SetAngle = -SetAngle;
	PID_calc(&PID_DM9015[0],DM9015_measure[0].TotalAngle,DM9015_measure[0].SetAngle);
	PID_calc(&PID_DM9015[1],DM9015_measure[1].TotalAngle,DM9015_measure[1].SetAngle);
	SlideBlock_ctrl.GivenValue[0] = PID_DM9015[0].out;
	SlideBlock_ctrl.GivenValue[1] = PID_DM9015[1].out;
	
}