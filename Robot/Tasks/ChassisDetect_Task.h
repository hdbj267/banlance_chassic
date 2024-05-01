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

//���̼��ṹ��
typedef struct
{
	uint32_t cnt_LoseCtrl;			//ʧ�ؼ��
	
	float slip_rk[CHECK_NUM];		//�򻬼��
	float slip_warning_value;
	float slip_rk_sum;
	uint8_t slip_num_rk_now;
	uint8_t isSlipped;
	
}chassis_detect_t;

//�򻬼�����ṹ��
typedef struct
{
	lpf_data_t LPF_1,LPF_2;		//��ͨ�˲�
	kf_data_t KF;							//�������
	float rk_sen;							//���ж�
	uint16_t phase;						//��λ
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

