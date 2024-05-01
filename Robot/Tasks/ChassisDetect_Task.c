#include "ChassisDetect_Task.h"
#include "ChassisControl_Task.h"
#include "ChassisSwitch_Task.h"
#include "Gimbal_Task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "can.h"
#include "remote_control.h"
#include "bsp_can.h"
#include "arm_math.h"
#include "INS_Task.h"
#include "Filter.h"
#include "Vofa_send.h"
#include "cmsis_os.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"



chassis_detect_t chassis_detect;

detect_slip_t detect_slip_speed;
detect_slip_t detect_slip_current;
detect_slip_t detect_slip_gyro;


/**
  * @brief  底盘检测任务
  */
void ChassisDetect_Task(void const * argument)
{
//	vTaskDelete(ChassisDetect_TaskHandle);
	vTaskDelay(2000);
	int32_t cnt=0;
	
	detect_slip_init(&detect_slip_speed		,0	,15.55f);
	detect_slip_init(&detect_slip_current	,40	,-0.025f);//-0.00629f
	detect_slip_init(&detect_slip_gyro		,0	,-10.0f);
	chassis_detect.slip_warning_value = 630 ;//要拿数值看着改，不要凭感觉
	while(1)
	{
///////////////////////////////////--Detect--//////////////////////////////////////////////////////////////////////////		
		//翻倒检测
		if((chassis_ctrl.ctrl_mode != MODE_STOP && chassis_ctrl.ctrl_mode != MODE_WEAK) && fabs(chassis_ctrl.angle_x)>0.63f)
		{
			chassis_detect.cnt_LoseCtrl++;
		}
		else if(chassis_detect.cnt_LoseCtrl>0)chassis_detect.cnt_LoseCtrl--;
		//打滑检测
		if(cnt % 2 == 0)
		{
			detect_slip_update(&detect_slip_speed,chassis_ctrl.speed_x);
			detect_slip_update(&detect_slip_current,0.5*(motor_chassis[0].given_current - motor_chassis[1].given_current));
			detect_slip_update(&detect_slip_gyro,chassis_ctrl.gyro_x);
			check_slipped(chassis_detect.slip_warning_value);
			debug_data_fp32[0] = detect_slip_speed.rk_adj;
			debug_data_fp32[1] = detect_slip_current.rk_adj;
			debug_data_fp32[2] = detect_slip_gyro.rk_adj;
			debug_data_fp32[3] = chassis_detect.slip_rk_sum;
		}
			
///////////////////////////////////--Detect--//////////////////////////////////////////////////////////////////////////		
		vTaskDelay(1);
		cnt++;
		if(cnt>=1000)cnt=0;
	}
}


/**
  * @brief  检测数据初始化
  * @param  检测数据结构体指针，相位，敏感度
  */
void detect_slip_init(detect_slip_t* data, uint16_t phase,	float sen)
{
	data->rk_sen = sen;
	fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
	data->phase = phase;
	LP_FilterInit(&data->LPF_1,fliter_num);
	LP_FilterInit(&data->LPF_2,fliter_num);
	Kalman_Init(&data->KF,0.1f,0.1f);
}


/**
  * @brief  检测数据更新
  * @param  检测数据结构体指针，检测数据输入
  */
void detect_slip_update(detect_slip_t* data, float new_data)
{
	LP_FilterCalc(&data->LPF_1,new_data * data->rk_sen);
	LP_FilterCalc(&data->LPF_2,data->LPF_1.out);
	KalmanFilterCalc(&data->KF,data->LPF_2.out);
	
	if(data->num_rk_now >= DETECT_NUM)data->num_rk_now = 0;
	data->KF_rk[data->num_rk_now]=data->KF.rk;
	int phase_num = data->num_rk_now - data->phase;
	if(phase_num<0)phase_num += DETECT_NUM;
	if(phase_num>=DETECT_NUM)phase_num -= DETECT_NUM;
	data->rk_adj = data->KF_rk[phase_num];
	data->num_rk_now++;
}


/**
  * @brief  判断是否打滑
  * @param  判断打滑预警值
  */
void check_slipped(float warning_value)
{
	float slip_rk = detect_slip_speed.rk_adj + detect_slip_current.rk_adj + detect_slip_gyro.rk_adj;
	if(chassis_detect.slip_num_rk_now >= CHECK_NUM)chassis_detect.slip_num_rk_now = 0;
	chassis_detect.slip_rk_sum -= chassis_detect.slip_rk[chassis_detect.slip_num_rk_now];
	chassis_detect.slip_rk[chassis_detect.slip_num_rk_now]=slip_rk;
	chassis_detect.slip_rk_sum += chassis_detect.slip_rk[chassis_detect.slip_num_rk_now];
	chassis_detect.slip_num_rk_now++;	
	
	if(chassis_detect.slip_rk_sum > chassis_detect.slip_warning_value)chassis_detect.isSlipped = 1;
	else chassis_detect.isSlipped = 0;
}