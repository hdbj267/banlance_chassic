#ifndef CHASSISDETECT_H
#define CHASSISDETECT_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "Filter.h"

#define DETECT_NUM 50
#define CHECK_NUM 50

//底盘检测结构体
typedef struct
{
	uint32_t cnt_LoseCtrl;			//失控检测
	
	float slip_rk[CHECK_NUM];		//打滑检测
	float slip_warning_value;
	float slip_rk_sum;
	uint8_t slip_num_rk_now;
	uint8_t isSlipped;
	
}chassis_detect_t;

//打滑检测对象结构体
typedef struct
{
	lpf_data_t LPF_1,LPF_2;		//低通滤波
	kf_data_t KF;							//卡方检测
	float rk_sen;							//敏感度
	uint16_t phase;						//相位
	float KF_rk[DETECT_NUM];
	uint8_t num_rk_now;
	float rk_adj;
}detect_slip_t;


extern chassis_detect_t chassis_detect;
extern detect_slip_t detect_slip_speed;
extern detect_slip_t detect_slip_current;
extern detect_slip_t detect_slip_gyro;


void detect_slip_init(detect_slip_t* data, uint16_t phase,	float sen);
	
void detect_slip_update(detect_slip_t* data, float new_data);
	
void check_slipped(float warning_value);

#endif

