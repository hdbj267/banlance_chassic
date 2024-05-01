#include "Gimbal_Task.h"
#include "ChassisSwitch_Task.h"
#include "ChassisControl_Task.h"
#include "ChassisDetect_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "CAN_receive.h"
#include "referee.h"

void Gimbal_Task(void const * argument)
{
	vTaskDelay(100);
	int32_t cnt=0;
	while(1)
	{
		//向上板发送裁判系统数据
		if(cnt % 20 == 0)
		{
			CAN_Gimbal_CMD(0x155	,Game_Robot_State.shooter_barrel_heat_limit
														,Game_Robot_State.shooter_barrel_cooling_value
														,30//Game_Robot_State.shooter_id1_17mm_speed_limit
														,Game_Robot_State.power_management_shooter_output
										);
		}

		vTaskDelay(1);
		cnt++;
		if(cnt>=1000)cnt=0;
	}
}

