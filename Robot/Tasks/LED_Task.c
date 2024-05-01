#include "LED_Task.h"
#include "FreeRTOS.h"
#include "task.h"


void LED_Task(void const * argument)
{
	TIM3->CCR1=1000;
	
	while(1)
	{
		HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_10);
		vTaskDelay(100);
	}
}
