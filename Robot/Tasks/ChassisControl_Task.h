#ifndef CHASSISCONTROL_TASK_H
#define CHASSISCONTROL_TASK_H

#include "struct_typedef.h"
#include "CAN_receive.h"
#include "gimbal_task.h"
#include "pid.h"
#include "remote_control.h"
#include "cmsis_os.h"
#include "arm_math.h"

//轮子直径
#define Diameter_weel 			0.160f
//定速控制
#define SPEED_LIMIT_60  		2.3f
#define SPEED_LIMIT_80  		2.5f
#define SPEED_LIMIT_100 		2.5f
#define SPEED_UNLIMIT 	 		2.5f
//加速控制
#define ACCEL_1_X 					0.7f          //前进加速度(正向)     
#define ACCEL_2_X 					0.35f					//前进加速度(反向) 
#define ACCEL_1_Y 					0.05f					//横移加速度(正向) 
#define ACCEL_2_Y					0.2f					//横移加速度(反向) 

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
#define PID_offset_MAX_IOUT 40.0f		//要是有点前前后后，一个是加大这里，也可以尝试加高物理模型质心的位置

#define Logistic( X, K, X0, L)	((L)/(1+pow(2.718282f,-(K)*((X)-(X0)))))//生物学过不，生长函数曲线

#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 3427

typedef enum
{
	CHASSIS_FRONT	= 0,
	CHASSIS_BACK	= 1,
	
}chassis_direction_e;

typedef struct
{
  const fp32 *INS_angle;						//陀螺仪欧拉角
	const fp32 *INS_gyro;							//陀螺仪角速度
	const fp32 *INS_accel;						//陀螺仪加速度
	const fp32 *INS_quat;							//陀螺仪四元数

	fp32 follow_gimbal_zero;					//跟随云台零点
	fp32 torque_const;								//转矩常数
	fp32 speed_x,target_speed_x;			//底盘速度
	fp32 pose_x,target_pose_x;				//底盘位移
	fp32 angle_x;											//底盘倾角
	fp32 gyro_x;											//底盘倾斜角速度
	fp32 angle_z;											//底盘旋转偏角
	fp32 omega_z;											//底盘旋转线速度
	fp32 accel_g;											//重力加速度
	
	int16_t	vx_rc,	vy_rc,	wz_rc;		//遥控速度
	uint8_t key_rc;										//遥控拨杆
	fp32 		vx_set,	vy_set,	wz_set;		//目标速度
	
	uint8_t chassis_direction;				//底盘正方向(CHASSIS_FRONT,CHASSIS_BACK)
	fp32 move_speed,move_direction;		//底盘平移速度，平移方向
	
	uint8_t ctrl_mode;								//底盘模式
	uint8_t ctrl_mode_last;						//上次底盘模式
	
	fp32 torque_speed;								//前进动力相应转矩
	fp32 torque_omega;								//旋转动力相应转矩
	fp32 torque_balance;							//平衡动力相应转矩
	fp32 GivenValue[2];								//底盘电机目标电流
	
	uint8_t flag_clear_pose;					//里程计停止清零标志(0,清零;1,停止清零)
	
}chassis_control_t;

//功率限制结构体
typedef struct
{
	float* control_obj;								//控制对象
	float no_limit;										//无限制允许输出
	float base_limit;									//基础限制允许输出
	float add_limit;									//增益限制允许输出
	float warning_buffer;							//缓冲能量预警值
	
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
  * @brief  数据范围限制
  * @param  输入数据,最小值,最大值
  * @retval 输出数据
  */
fp32 limit(float data, float min, float max);

/**
  * @brief  死区限制
  * @param  输入信号;死区
  * @retval 输出信号
  */
fp32 deadband_limit(float rc_in , float deadband );

/**
  * @brief  底盘状态更新
  */
void chassis_state_update(void);

/**
  * @brief  底盘状态清除
  * @param  none
  * @retval none
  */
void chassis_state_clear(void);
	
/**
  * @brief  速度控制
  * @param  输入速度,运动方向
  * @retval 输出速度
  */
float speed_control(float speed_in,float direction);

/**
  * @brief  梯形控制
  * @param  反馈；设定；加速度
  * @retval 输出值
  */
float ramp_control(float ref ,float set,float accel);
	
/**
  * @brief  速度灵敏度调节
  * @retval 速度灵敏度
  */
float speed_sen_adjust(void);
	
/**
  * @brief  裁判系统功率限制
  * @param  调节对象指针
  */
void Chassis_RefereePowerControl(chassis_power_t *power_obj);
	
/**
  * @brief  电容功率限制
  * @param  调节对象指针
  */
void Chassis_CapPowerControl(chassis_power_t *power_obj);

/**
  * @brief  底盘方向偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_err_get(float target_ang);

/**
  * @brief  底盘方向最小偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_min_err_get(float target_ang);

#endif

