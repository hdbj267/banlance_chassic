#ifndef CHASSISCONTROL_TASK_H
#define CHASSISCONTROL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "arm_math.h"

//����ֱ��
#define Diameter_weel 			0.160f
//���ٿ���
#define SPEED_LIMIT_60  		2.3f
#define SPEED_LIMIT_80  		2.5f
#define SPEED_LIMIT_100 		2.5f
#define SPEED_UNLIMIT 	 		2.5f
//���ٿ���
#define ACCEL_1_X 					0.7f          //ǰ�����ٶ�(����)     
#define ACCEL_2_X 					0.35f					//ǰ�����ٶ�(����) 
#define ACCEL_1_Y 					0.05f					//���Ƽ��ٶ�(����) 
#define ACCEL_2_Y					0.2f					//���Ƽ��ٶ�(����) 

#define MF9025_MOTOR_SPEED_PID_KP 2.98142f
#define MF9025_MOTOR_SPEED_PID_KI 0.0f
#define MF9025_MOTOR_SPEED_PID_KD 0.0f
#define MF9025_MOTOR_SPEED_PID_MAX_OUT 2000.0f
#define MF9025_MOTOR_SPEED_PID_MAX_IOUT 0.0f

#define PID_LQR_k1_KP 2.886751f
#define PID_LQR_k1_KI 0.0f
#define PID_LQR_k1_KD 0.0f
#define PID_LQR_k1_MAX_OUT 120.0f
#define PID_LQR_k1_MAX_IOUT 0.0f

#define PID_LQR_k2_KP 9.382265f
#define PID_LQR_k2_KI 0.01f
#define PID_LQR_k2_KD 0.0f
#define PID_LQR_k2_MAX_OUT 250.0f
#define PID_LQR_k2_MAX_IOUT 20.0f

#define PID_LQR_k3_KP 22.896976f		
#define PID_LQR_k3_KI 0.0f
#define PID_LQR_k3_KD 0.0f
#define PID_LQR_k3_MAX_OUT 16000.0f
#define PID_LQR_k3_MAX_IOUT 8000.0f

#define PID_LQR_k4_KP 3.662144f         
#define PID_LQR_k4_KI 0.0f
#define PID_LQR_k4_KD 0.0f
#define PID_LQR_k4_MAX_OUT 16000.0f
#define PID_LQR_k4_MAX_IOUT 0.0f			

#define PID_LQR_k5_KP 2.5f
#define PID_LQR_k5_KI 0.0f
#define PID_LQR_k5_KD 0.0f//20
#define PID_LQR_k5_MAX_OUT 12000.0f
#define PID_LQR_k5_MAX_IOUT 2000.0f	

#define PID_LQR_k6_KP 10.0f
#define PID_LQR_k6_KI 0.0f
#define PID_LQR_k6_KD 0.0f
#define PID_LQR_k6_MAX_OUT 12000.0f
#define PID_LQR_k6_MAX_IOUT 2000.0f

#define PID_offset_KP 1.0f
#define PID_offset_KI 2.0f
#define PID_offset_KD 0.0f
#define PID_offset_MAX_OUT 2000.0f		
#define PID_offset_MAX_IOUT 40.0f		//Ҫ���е�ǰǰ���һ���ǼӴ����Ҳ���Գ��ԼӸ�����ģ�����ĵ�λ��

#define Logistic( X, K, X0, L)	((L)/(1+pow(2.718282f,-(K)*((X)-(X0)))))//����ѧ������������������

#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 3427

typedef enum
{
	CHASSIS_FRONT	= 0,
	CHASSIS_BACK	= 1,
	
}chassis_direction_e;

typedef struct
{
  const fp32 *INS_angle;						//������ŷ����
	const fp32 *INS_gyro;							//�����ǽ��ٶ�
	const fp32 *INS_accel;						//�����Ǽ��ٶ�
	const fp32 *INS_quat;							//��������Ԫ��

	fp32 follow_gimbal_zero;					//������̨���
	fp32 torque_const;								//ת�س���
	fp32 speed_x,target_speed_x;			//�����ٶ�
	fp32 pose_x,target_pose_x;				//����λ��
	fp32 angle_x;											//�������
	fp32 gyro_x;											//������б���ٶ�
	fp32 angle_z;											//������תƫ��
	fp32 omega_z;											//������ת���ٶ�
	fp32 accel_g;											//�������ٶ�
	
	int16_t	vx_rc,	vy_rc,	wz_rc;		//ң���ٶ�
	uint8_t key_rc;										//ң�ز���
	fp32 		vx_set,	vy_set,	wz_set;		//Ŀ���ٶ�
	
	uint8_t chassis_direction;				//����������(CHASSIS_FRONT,CHASSIS_BACK)
	fp32 move_speed,move_direction;		//����ƽ���ٶȣ�ƽ�Ʒ���
	
	uint8_t ctrl_mode;								//����ģʽ
	uint8_t ctrl_mode_last;						//�ϴε���ģʽ
	
	fp32 torque_speed;								//ǰ��������Ӧת��
	fp32 torque_omega;								//��ת������Ӧת��
	fp32 torque_balance;							//ƽ�⶯����Ӧת��
	fp32 GivenValue[2];								//���̵��Ŀ�����
	
	uint8_t flag_clear_pose;					//��̼�ֹͣ�����־(0,����;1,ֹͣ����)
	
}chassis_control_t;

//�������ƽṹ��
typedef struct
{
	float* control_obj;								//���ƶ���
	float no_limit;										//�������������
	float base_limit;									//���������������
	float add_limit;									//���������������
	float warning_buffer;							//��������Ԥ��ֵ
	
}chassis_power_t;

extern chassis_control_t chassis_ctrl;
extern chassis_power_t PowerLimit_speed;
extern chassis_power_t PowerLimit_balance;
extern chassis_power_t PowerLimit_k5;
extern osThreadId ChassisControl_TaskHandle;
extern osThreadId ChassisDetect_TaskHandle;
extern osThreadId ChassisSwitch_TaskHandle;
extern connect_t connect_data;

extern pid_type_def PID_MF9025_speed[2];
extern pid_type_def PID_LQR_k1;
extern pid_type_def PID_LQR_k2;
extern pid_type_def PID_LQR_k3;
extern pid_type_def PID_LQR_k4;
extern pid_type_def PID_LQR_k5;
extern pid_type_def PID_LQR_k6;
extern pid_type_def PID_offset;

/**
  * @brief  ���ݷ�Χ����
  * @param  ��������,��Сֵ,���ֵ
  * @retval �������
  */
fp32 limit(float data, float min, float max);

/**
  * @brief  ��������
  * @param  �����ź�;����
  * @retval ����ź�
  */
fp32 deadband_limit(float rc_in , float deadband );

/**
  * @brief  ����״̬����
  */
void chassis_state_update(void);

/**
  * @brief  ����״̬���
  * @param  none
  * @retval none
  */
void chassis_state_clear(void);
	
/**
  * @brief  �ٶȿ���
  * @param  �����ٶ�,�˶�����
  * @retval ����ٶ�
  */
float speed_control(float speed_in,float direction);

/**
  * @brief  ���ο���
  * @param  �������趨�����ٶ�
  * @retval ���ֵ
  */
float ramp_control(float ref ,float set,float accel);
	
/**
  * @brief  �ٶ������ȵ���
  * @retval �ٶ�������
  */
float speed_sen_adjust(void);
	
/**
  * @brief  ����ϵͳ��������
  * @param  ���ڶ���ָ��
  */
void Chassis_RefereePowerControl(chassis_power_t *power_obj);
	
/**
  * @brief  ���ݹ�������
  * @param  ���ڶ���ָ��
  */
void Chassis_CapPowerControl(chassis_power_t *power_obj);

/**
  * @brief  ���̷���ƫ���ȡ
  * @param  Ŀ�귽��
  * @retval ����ƫ��
  */
float angle_z_err_get(float target_ang);

/**
  * @brief  ���̷�����Сƫ���ȡ
  * @param  Ŀ�귽��
  * @retval ����ƫ��
  */
float angle_z_min_err_get(float target_ang);

#endif

