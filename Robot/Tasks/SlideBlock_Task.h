#ifndef SLIDEBLOCK_TASK_H
#define SLIDEBLOCK_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern osThreadId SlideBlock_TaskHandle;

typedef enum
{
	BLOCK_WEAK 		= 0,
	BLOCK_READY 	= 1,
	BLOCK_NORMAL	= 2,
	
}SlideBlock_mode_e;

typedef struct
{
	uint8_t ctrl_mode;
	uint8_t ctrl_mode_last;
	uint8_t isReady;
	float angle_x;
	float target_pos;
	int16_t GivenValue[2];
	
}SlideBlock_control_t;

	
extern SlideBlock_control_t SlideBlock_ctrl;




/**
  * @brief  滑块运动模式
  */
void SlideBlock_normal();
	
/**
  * @brief  滑块无力模式
  */
void SlideBlock_weak();
	
/**
  * @brief  滑块准备模式
  */
void SlideBlock_ready();



#endif
