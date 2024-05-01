

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
#include "referee.h"


pid_type_def PID_MF9025_speed[2];
pid_type_def PID_LQR_k1;
pid_type_def PID_LQR_k2;
pid_type_def PID_LQR_k3;
pid_type_def PID_LQR_k4;
pid_type_def PID_LQR_k5;
pid_type_def PID_LQR_k6;
pid_type_def PID_offset;

float chackset;
float chackfbd;

float chassis_MF9025_pid[3] = {MF9025_MOTOR_SPEED_PID_KP, MF9025_MOTOR_SPEED_PID_KI, MF9025_MOTOR_SPEED_PID_KD};
float pid_LQR_k1[3] = {PID_LQR_k1_KP, PID_LQR_k1_KI, PID_LQR_k1_KD};
float pid_LQR_k2[3] = {PID_LQR_k2_KP, PID_LQR_k2_KI, PID_LQR_k2_KD};
float pid_LQR_k3[3] = {PID_LQR_k3_KP, PID_LQR_k3_KI, PID_LQR_k3_KD};
float pid_LQR_k4[3] = {PID_LQR_k4_KP, PID_LQR_k4_KI, PID_LQR_k4_KD};
float pid_LQR_k5[3] = {PID_LQR_k5_KP, PID_LQR_k5_KI, PID_LQR_k5_KD};
float pid_LQR_k6[3] = {PID_LQR_k6_KP, PID_LQR_k6_KI, PID_LQR_k6_KD};
float pid_offset[3] = {PID_offset_KP, PID_offset_KI, PID_offset_KD};

chassis_control_t chassis_ctrl;

//定义功率限制对象
chassis_power_t PowerLimit_speed =		{	.control_obj = &chassis_ctrl.torque_speed,
																				.warning_buffer = 30.0f,
																				.no_limit 	= 4000.0f,
																				.base_limit	= 2000.0f,
																				.add_limit 	= 2000.0f,	};

chassis_power_t PowerLimit_balance =	{	.control_obj = &chassis_ctrl.torque_balance,
																				.warning_buffer = 30.0f,
																				.no_limit 	= 4000.0f,
																				.base_limit	= 2000.0f,
																				.add_limit 	= 2000.0f,	};

chassis_power_t PowerLimit_k5 =				{	.control_obj = &PID_LQR_k5.out,
																				.warning_buffer = 30.0f,
																				.no_limit 	= 2.0f,
																				.base_limit	= 1.0f,
																				.add_limit 	= 1.0f,	};
float err;

/**
  * @brief  底盘控制任务
  */
void ChassisControl_Task(void const * argument)
{
	vTaskDelay(2000);
	
	chassis_ctrl.torque_const = 195.3125;		//转矩系数
	
	chassis_ctrl.INS_accel = get_INS_accel_point();
	chassis_ctrl.INS_angle = get_INS_angle_point();
	chassis_ctrl.INS_gyro = get_gyro_data_point();
	chassis_ctrl.INS_quat = get_INS_quat_point();
	
	PID_init(&PID_MF9025_speed[0], PID_POSITION, chassis_MF9025_pid, MF9025_MOTOR_SPEED_PID_MAX_OUT, MF9025_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&PID_MF9025_speed[1], PID_POSITION, chassis_MF9025_pid, MF9025_MOTOR_SPEED_PID_MAX_OUT, MF9025_MOTOR_SPEED_PID_MAX_IOUT);
	PID_init(&PID_LQR_k1, PID_POSITION, pid_LQR_k1, PID_LQR_k1_MAX_OUT, PID_LQR_k1_MAX_IOUT);
	PID_init(&PID_LQR_k2, PID_POSITION, pid_LQR_k2, PID_LQR_k2_MAX_OUT, PID_LQR_k2_MAX_IOUT);
	PID_init(&PID_LQR_k3, PID_POSITION, pid_LQR_k3, PID_LQR_k3_MAX_OUT, PID_LQR_k3_MAX_IOUT);
	PID_init(&PID_LQR_k4, PID_POSITION, pid_LQR_k4, PID_LQR_k4_MAX_OUT, PID_LQR_k4_MAX_IOUT);
	PID_init(&PID_LQR_k5, PID_POSITION, pid_LQR_k5, PID_LQR_k5_MAX_OUT, PID_LQR_k5_MAX_IOUT);
	PID_init(&PID_LQR_k6, PID_POSITION, pid_LQR_k6, PID_LQR_k6_MAX_OUT, PID_LQR_k6_MAX_IOUT);
	PID_init(&PID_offset, PID_POSITION, pid_offset, PID_offset_MAX_OUT, PID_offset_MAX_IOUT);
	while(1)
	{
		chassis_state_update();
	    err = deadband_limit(angle_z_err_get(-chassis_ctrl.move_direction+(motor_gimbal.ecd / 1303.7973f)),0.02f);
		if(chassis_ctrl.ctrl_mode == MODE_STATIC)
		{
			chassis_static();
		}
		if(chassis_ctrl.ctrl_mode == MODE_NORMAL)
		{
			chassis_normal();
		}
		if(chassis_ctrl.ctrl_mode == MODE_WEAK)
		{
			chassis_weak();
		}
		if(chassis_ctrl.ctrl_mode == MODE_STOP)	
		{
			chassis_stop();
		}
		if(chassis_ctrl.ctrl_mode == MODE_BALANCE)	
		{
			chassis_balance();
		}
		if(chassis_ctrl.ctrl_mode == MODE_SLIPPED)	
		{
			chassis_slipped();
		}
		
		if(connect_data.can2_rc_ctrl.work_mode==ROBOT_ROTATE_STOP_MODE||connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_Q)
		{	
			chassis_ctrl.wz_rc=550;
		}
		else 
		{	
			if( err>=-0.70 &err<=-0.50 )		//速度越高，值或许越大
			{
			chassis_ctrl.wz_rc=0;
			}
		}
		
//		if(connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_C) 
//		{	
//			if( err>=-0.50 &err<=0.20 )		//速度越高，值或许越大
//			{
//			chassis_ctrl.wz_rc=0;
//			}
//		}
//		 else if((connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_Q) && fabs(chassis_ctrl.angle_x) < 0.040)
//		{	
//			chassis_ctrl.wz_rc=550;
//		}
		
		chackset=PID_LQR_k5.set;
		chackfbd=PID_LQR_k5.fdb;
		CAN_cmd_chassis(chassis_ctrl.GivenValue[0],chassis_ctrl.GivenValue[1]);
		
		vTaskDelay(1);
		
//		CAN_Cap_CMD(Game_Robot_State.chassis_power_limit,Power_Heat_Data.chassis_power,Power_Heat_Data.chassis_power_buffer,0);
	}
}

/**
  * @brief  死区限制
  * @param  输入信号;死区
  * @retval 输出信号
  */
fp32 deadband_limit(float rc_in , float deadband )
{
	if(fabs(rc_in)<deadband)return 0;
	return rc_in;
}

/**
  * @brief  转角限制到±PI
  * @param  输入转角
  * @retval 输出转角
  */
fp32 limit_pi(fp32 in)
{
	while(in < -PI || in > PI)
	{
		if (in < -PI)
			in = in + PI + PI;
		if (in > PI)
			in = in - PI - PI;
	}
	return in;
}

/**
  * @brief  数据范围限制
  * @param  输入数据,最小值,最大值
  * @retval 输出数据
  */
fp32 limit(float data, float min, float max)
{
	if (data >= max)
		return max;
	if (data <= min)
		return min;
	return data;
}

/**
  * @brief  重力加速度测量
  * @param  none
  * @retval 重力加速度
  */
fp32 accel_g_solve()
{
	fp32 q0 = chassis_ctrl.INS_quat[0];
	fp32 q1 = chassis_ctrl.INS_quat[1];
	fp32 q2 = chassis_ctrl.INS_quat[2];
	fp32 q3 = chassis_ctrl.INS_quat[3];
	fp32 a0 = chassis_ctrl.INS_accel[0];
	fp32 a1 = chassis_ctrl.INS_accel[1];
	fp32 a2 = chassis_ctrl.INS_accel[2];
	return a0 * (2 * q1 * q3 - 2 * q0 * q2) + a1 * (2 * q2 * q3 + 2 * q0 * q1) + a2 * (1 - 2 * q1 * q1 - 2 * q2 * q2);
}


kf_data_t kf2;
/**
  * @brief  底盘状态更新
  * @param  none
  * @retval none
  */
void chassis_state_update()
{
	chassis_ctrl.angle_x=chassis_ctrl.INS_angle[1];
	chassis_ctrl.gyro_x=chassis_ctrl.INS_gyro[0];
	chassis_ctrl.accel_g = accel_g_solve();
	chassis_ctrl.speed_x = -(motor_chassis[0].speed - motor_chassis[1].speed) /57.29578f*Diameter_weel/2.0f/2.0f + chassis_ctrl.gyro_x * 0.03f;
	chassis_ctrl.omega_z = 	(motor_chassis[0].speed + motor_chassis[1].speed) /57.29578f*Diameter_weel/2.0f/2.0f;//180/pi
	chassis_ctrl.pose_x += chassis_ctrl.speed_x * 0.001f;
	//设置运动速度
	if(connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_W)
		chassis_ctrl.vx_set = 360 * speed_sen_adjust();
	else if(connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_S)
		chassis_ctrl.vx_set = -210 * speed_sen_adjust();
	else
		chassis_ctrl.vx_set = 0.8 * -connect_data.can2_rc_ctrl.rc.ch2 * speed_sen_adjust();
	
	if(connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_A)
		chassis_ctrl.vy_set = 300 * speed_sen_adjust();
	else if(connect_data.can2_rc_ctrl.mouse.key==KEY_PRESSED_OFFSET_D)
		chassis_ctrl.vy_set = -300 * speed_sen_adjust();
	else
		chassis_ctrl.vy_set = 0.5 * -connect_data.can2_rc_ctrl.rc.ch3 * speed_sen_adjust();
	
	chassis_ctrl.wz_set = chassis_ctrl.wz_rc * speed_sen_adjust();
	
	chassis_ctrl.move_speed = sqrt(chassis_ctrl.vx_set*chassis_ctrl.vx_set + chassis_ctrl.vy_set*chassis_ctrl.vy_set);
	limit(chassis_ctrl.move_speed, -660 * speed_sen_adjust(), 660 * speed_sen_adjust());
	if(chassis_ctrl.chassis_direction == CHASSIS_BACK)chassis_ctrl.move_speed *= -1;
	chassis_ctrl.move_direction = atan2f(chassis_ctrl.vy_set,chassis_ctrl.vx_set);
}

/**
  * @brief  底盘状态清除
  * @param  none
  * @retval none
  */
void chassis_state_clear()
{
	chassis_ctrl.pose_x = 0;
	chassis_ctrl.target_pose_x=0;
	chassis_ctrl.target_speed_x=0;
	PID_clear(&PID_LQR_k1);
	PID_clear(&PID_LQR_k2);
	PID_clear(&PID_LQR_k3);
	PID_clear(&PID_LQR_k4);
	PID_clear(&PID_LQR_k5);
	PID_clear(&PID_LQR_k6);
	PID_clear(&PID_offset);
}


/**
  * @brief  速度控制
  * @param  输入速度,运动方向
  * @retval 输出速度
  */
float speed_control(float speed_in,float direction)
{
	fp32 accel_K1;
	fp32 accel_K2;
	if(fabs(direction)>1.39&&fabs(direction)<1.75)
	{
		accel_K1 = ACCEL_1_Y;
		accel_K2 = ACCEL_2_Y;
	}
	else 
	{
		accel_K1 = ACCEL_1_X;
		accel_K2 = ACCEL_2_X;//设置往回拉的加速度
	}
	if (speed_in >= 0)
	{
		fp32 accel_1 = accel_K1*(speed_in - chassis_ctrl.speed_x);
		fp32 accel_2 = accel_K2*(speed_in - chassis_ctrl.speed_x);
		if (chassis_ctrl.speed_x >= 0)
		{
			return chassis_ctrl.speed_x + accel_1;
		}
		else if (chassis_ctrl.speed_x < 0)
			return chassis_ctrl.speed_x + accel_2;
	}
	if (speed_in < 0)
	{
		fp32 accel_1 = -accel_K1*(speed_in - chassis_ctrl.speed_x);
		fp32 accel_2 = -accel_K2*(speed_in - chassis_ctrl.speed_x);
		if (chassis_ctrl.speed_x < 0)
		{
			return chassis_ctrl.speed_x - accel_1;
		}
		else if (chassis_ctrl.speed_x >= 0)
			return chassis_ctrl.speed_x - accel_2;
	}
}


/**
  * @brief  裁判系统功率限制
  * @param  调节对象指针
  */
void Chassis_RefereePowerControl(chassis_power_t *power_obj)
{
	float total_current_limit = 0.0f;
	float total_current       = 0.0f;
	float power_scale         = 0.0f;
	float current_scale       = 0.0f;
			
	if(Game_Robot_State.chassis_power_limit == 0 || Game_Robot_State.chassis_power_limit == 65535)
	{
		total_current_limit = power_obj -> no_limit;
	}
  else
	{
		if(Power_Heat_Data.chassis_power_buffer < power_obj -> warning_buffer)
		{
			if(Power_Heat_Data.chassis_power_buffer > 5.0f)
			{
					power_scale = Power_Heat_Data.chassis_power_buffer / power_obj -> warning_buffer;
			}
			else
			{
					power_scale = 5.0f / power_obj -> warning_buffer;
			}
			total_current_limit = power_obj -> base_limit * power_scale;
		}
		else
		{
			if(Power_Heat_Data.chassis_power > Game_Robot_State.chassis_power_limit * 0.75f)
			{
				if(Power_Heat_Data.chassis_power < Game_Robot_State.chassis_power_limit)
				{
					power_scale = (Game_Robot_State.chassis_power_limit - Power_Heat_Data.chassis_power) / 
					(Game_Robot_State.chassis_power_limit - Game_Robot_State.chassis_power_limit*0.75f);	
				}
				else
				{
					power_scale = 0.0f;
				}
				
				total_current_limit = power_obj -> base_limit + power_obj -> add_limit * power_scale;
			}
			else
			{
				total_current_limit = power_obj -> base_limit + power_obj -> add_limit;
			}
		}
	}
	
	total_current += fabs(*(power_obj -> control_obj));

  if(total_current > total_current_limit)
  {
		current_scale = total_current_limit / total_current;
		*(power_obj -> control_obj) *= current_scale;
  }
}

/**
  * @brief  电容功率限制
  * @param  调节对象指针
  */
void Chassis_CapPowerControl(chassis_power_t *power_obj)
{
	float total_current_limit = 0.0f;
	float total_current       = 0.0f;
	float power_scale         = 0.0f;
	float current_scale       = 0.0f;
	
	if(Chassis_Capacitance == 1)
	{
		return;
	}
  else
	{
		if(Chassis_Capacitance < 0.35f)
		{
			power_scale = Chassis_Capacitance / 0.35f;
			total_current_limit = power_obj -> base_limit * power_scale;
		}
		else
		{
			total_current_limit = power_obj -> no_limit;
		}
	}
	
	total_current += fabs(*(power_obj -> control_obj));

  if(total_current > total_current_limit)
  {
		current_scale = total_current_limit / total_current;
		*(power_obj -> control_obj) *= current_scale;
  }
}



/**
  * @brief  速度灵敏度调节
  * @retval 速度灵敏度
  */
float speed_sen_adjust()
{
	float speed_sen;
	if(Game_Robot_State.chassis_power_limit == 0)
	{
		speed_sen = SPEED_UNLIMIT/660.0f;
	}
	else if(Game_Robot_State.chassis_power_limit <= 60)
	{
		speed_sen = SPEED_LIMIT_60/660.0f;
	}
	else if(Game_Robot_State.chassis_power_limit <= 80)
	{
		speed_sen = SPEED_LIMIT_80/660.0f;
	}
	else if(Game_Robot_State.chassis_power_limit <= 100)
	{
		speed_sen = SPEED_LIMIT_100/660.0f;
	}
	else if(Game_Robot_State.chassis_power_limit <= 65535)
	{
		speed_sen = SPEED_UNLIMIT/660.0f;
	}
		
	return speed_sen;
}

	
/**
  * @brief  底盘方向偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_err_get(float target_ang)
{
	float AngErr_front,AngErr_back;
	AngErr_front=limit_pi(chassis_ctrl.follow_gimbal_zero / 1303.7973f - target_ang);
	AngErr_back	=limit_pi((chassis_ctrl.follow_gimbal_zero + 4096) / 1303.7973f - target_ang);
	if(fabs(AngErr_front)>fabs(AngErr_back))
	{
		chassis_ctrl.chassis_direction = CHASSIS_BACK;
		return AngErr_back;
	}
	else 
	{
		chassis_ctrl.chassis_direction = CHASSIS_FRONT;
		return AngErr_front;
	}
}

/**
  * @brief  底盘方向最小偏差获取
  * @param  目标方向
  * @retval 方向偏差
  */
float angle_z_min_err_get(float target_ang)
{
	float AngErr_front,AngErr_back,AngErr_left,AngErr_right;
	AngErr_front=limit_pi(chassis_ctrl.follow_gimbal_zero / 1303.7973f - target_ang);
	AngErr_back	=limit_pi((chassis_ctrl.follow_gimbal_zero + 4096) / 1303.7973f - target_ang);
	AngErr_left =limit_pi((chassis_ctrl.follow_gimbal_zero + 2048) / 1303.7973f - target_ang);
	AngErr_right=limit_pi((chassis_ctrl.follow_gimbal_zero - 2048) / 1303.7973f - target_ang);
	float min_err = AngErr_front;
	if(fabs(min_err)>fabs(AngErr_back))min_err=AngErr_back;
	if(fabs(min_err)>fabs(AngErr_left))min_err=AngErr_left;
	if(fabs(min_err)>fabs(AngErr_right))min_err=AngErr_right;
	return min_err;
}

/**
  * @brief  梯形控制
  * @param  反馈；设定；加速度
  * @retval 输出值
  */
float ramp_control(float ref ,float set,float accel)
{
	fp32 ramp = limit(accel,0,1)*(set - ref);
	return ref + ramp;
}