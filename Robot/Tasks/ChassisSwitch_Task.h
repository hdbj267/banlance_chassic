#ifndef CHASSISSWITCH_TASK_H
#define CHASSISSWITCH_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"

typedef enum
{
	ROBOT_CALI_MODE = 0,     //调试模式
	ROBOT_INIT_MODE,         //初始化
	ROBOT_INIT_END_MODE,     //初始化结束切换点
	ROBOT_COMMON_MODE,       //普通底盘跟随模式
	ROBOT_ROTATE_STOP_MODE,  //静止小陀螺
	ROBOT_WEAK_MODE,         //底盘倒下
	ROBOT_ERROR_MODE,        //错误
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
  * @brief  底盘模式切换
  * @param  新底盘模式
  */
void mode_switch_chassis(uint8_t mode);

/**
  * @brief  滑块模式切换
  * @param  新底盘模式
  */
void mode_switch_block(uint8_t mode);

/**
  * @brief  底盘静态模式
  */
void chassis_static(void);

/**
  * @brief  底盘正常模式
  */
void chassis_normal(void);

/**
  * @brief  底盘无力模式
  */
void chassis_weak(void);

/**
  * @brief  底盘抱死模式
  */
void chassis_stop(void);

/**
  * @brief  底盘平衡模式
  */
void chassis_balance(void);

/**
  * @brief  底盘打滑模式
  */
void chassis_slipped(void);

#endif

