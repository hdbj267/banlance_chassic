#ifndef CHASSISSWITCH_TASK_H
#define CHASSISSWITCH_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"

typedef enum
{
	ROBOT_CALI_MODE = 0,     //����ģʽ
	ROBOT_INIT_MODE,         //��ʼ��
	ROBOT_INIT_END_MODE,     //��ʼ�������л���
	ROBOT_COMMON_MODE,       //��ͨ���̸���ģʽ
	ROBOT_ROTATE_STOP_MODE,  //��ֹС����
	ROBOT_WEAK_MODE,         //���̵���
	ROBOT_ERROR_MODE,        //����
}robot_work_mode_e;

typedef enum
{
	MODE_STATIC 		= 0,
	MODE_NORMAL 		= 1,
	MODE_WEAK 			= 2,
	MODE_STOP 			= 3,
	MODE_BALANCE 		= 4,
	MODE_SLIPPED		= 5,
}chassis_mode_e;


/**
  * @brief  ����ģʽ�л�
  * @param  �µ���ģʽ
  */
void mode_switch_chassis(uint8_t mode);

/**
  * @brief  ����ģʽ�л�
  * @param  �µ���ģʽ
  */
void mode_switch_block(uint8_t mode);

/**
  * @brief  ���̾�̬ģʽ
  */
void chassis_static(void);

/**
  * @brief  ��������ģʽ
  */
void chassis_normal(void);

/**
  * @brief  ��������ģʽ
  */
void chassis_weak(void);

/**
  * @brief  ���̱���ģʽ
  */
void chassis_stop(void);

/**
  * @brief  ����ƽ��ģʽ
  */
void chassis_balance(void);

/**
  * @brief  ���̴�ģʽ
  */
void chassis_slipped(void);

#endif

