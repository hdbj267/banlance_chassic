#include "ChassisSwitch_Task.h"
#include "ChassisControl_Task.h"
#include "ChassisDetect_Task.h"
#include "SlideBlock_Task.h"
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

#define CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO 3427  //ͷ�ļ�Ҳ��


/**
  * @brief  �����л�����
  */
void ChassisSwitch_Task(void const * argument)
{
	vTaskDelay(1000);
	mode_switch_chassis(MODE_STOP);
	chassis_ctrl.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	while(1)
	{
		vTaskDelay(3);
		//ң��������λ��
		if((connect_data.can2_rc_ctrl.work_mode==ROBOT_COMMON_MODE||connect_data.can2_rc_ctrl.control_mode==1)&&connect_data.receive_rc_data_flag==1)
		{
			//������ǳ���75��
			if (fabs(chassis_ctrl.angle_x)>75.0f/57.3f)
			{
				mode_switch_chassis(MODE_STOP);
				continue;
			}
			//����������������35��
//			if (Chassis_Capacitance<0.35)
//			{
//				mode_switch_chassis(MODE_STOP);
//				while(Chassis_Capacitance<0.50) vTaskDelay(10);
//				continue;
//			}
			//�Ƿ�ʧ��
			if (chassis_detect.cnt_LoseCtrl>350)
			{
	 			mode_switch_chassis(MODE_STOP);
				while(fabs(chassis_ctrl.speed_x)<0.20)vTaskDelay(1);
				vTaskDelay(3000);
				chassis_detect.cnt_LoseCtrl=0;
				continue;
			}
			//�Ƿ����
			if (chassis_ctrl.accel_g < 5.0f)
			{
				mode_switch_chassis(MODE_BALANCE);
				continue;
			}
			//��ǰ���ٶ�	
			if(chassis_ctrl.move_speed == 0)
			{
				//���̴��㵹״̬�ָ�
				if((chassis_ctrl.ctrl_mode == MODE_STOP || chassis_ctrl.ctrl_mode == MODE_WEAK) && fabs(chassis_ctrl.angle_x)>10.0/57.3)
				{
//					mode_switch_block(BLOCK_READY);
//					SlideBlock_ctrl.angle_x = chassis_ctrl.angle_x;
//					vTaskDelay(200);
//					mode_switch_chassis(MODE_STATIC);	
//					vTaskDelay(300);
//					mode_switch_block(BLOCK_WEAK);
					PID_LQR_k1.Kp = 1.445154;	//̫���ײ��
				}
				else
				{
					PID_LQR_k1.Kp = PID_LQR_k1_KP;//վ��������ʱ����Դ��
				}
				mode_switch_chassis(MODE_STATIC);	
//				if(chassis_ctrl.flag_clear_pose == 1)mode_switch_block(BLOCK_WEAK);
				//�Ƿ��
				if(chassis_detect.isSlipped == 1)
				{
					mode_switch_chassis(MODE_SLIPPED);
					for(uint16_t i=0;i<50;i++)
					{
						if(chassis_detect.slip_rk_sum <= 150)break;
						vTaskDelay(2);
					}
				}
				continue;
			}
			mode_switch_chassis(MODE_NORMAL);
			mode_switch_block(BLOCK_NORMAL);
			if(fabs(chassis_ctrl.move_direction)>1.39&&fabs(chassis_ctrl.move_direction)<1.75)
				mode_switch_block(BLOCK_WEAK);
		}
		else
		{
			if(chassis_ctrl.move_speed == 0)mode_switch_chassis(MODE_WEAK);
			else mode_switch_chassis(MODE_STOP);
			chassis_ctrl.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
		}
	
	}
}


/**
  * @brief  ����ģʽ�л�
  * @param  �µ���ģʽ
  */
void mode_switch_chassis(uint8_t mode)
{
	if(mode == chassis_ctrl.ctrl_mode)return;
	else 
	{
		chassis_ctrl.ctrl_mode_last = chassis_ctrl.ctrl_mode;
		chassis_ctrl.ctrl_mode = mode;
	}
}


/**
  * @brief  ����ģʽ�л�
  * @param  �µ���ģʽ
  */
void mode_switch_block(uint8_t mode)
{
	if(mode == SlideBlock_ctrl.ctrl_mode)return;
	else 
	{
		SlideBlock_ctrl.ctrl_mode_last = chassis_ctrl.ctrl_mode;
		SlideBlock_ctrl.ctrl_mode = mode;
	}
}

float K3 = 32.694793;
float K4 = 7.714374;
/**
  * @brief  ���̾�̬ģʽ
  */
void chassis_static()
{
	PID_LQR_k2.Ki = 0.0f;
	PID_LQR_k2.max_iout = 0.0f;
	PID_clear(&PID_LQR_k2);
	if(fabs(chassis_ctrl.speed_x)>0.4f&&fabs(chassis_ctrl.speed_x)<0.45f&&chassis_ctrl.flag_clear_pose == 0)
	{
		chassis_ctrl.target_pose_x=chassis_ctrl.pose_x;
		chassis_ctrl.flag_clear_pose = 1;
	}
	
	chassis_ctrl.target_speed_x = 0;
	chassis_ctrl.angle_z=deadband_limit(angle_z_min_err_get(motor_gimbal.ecd / 1303.7973f),0.02f);//��Сת��8129/2pi=1303.79
//	chassis_ctrl.angle_z=deadband_limit(angle_z_err_get(-chassis_ctrl.move_direction+(motor_gimbal.ecd / 1303.7973f)),0.02f);	
	PID_calc(&PID_LQR_k1, chassis_ctrl.pose_x, chassis_ctrl.target_pose_x);
	PID_calc(&PID_LQR_k2, chassis_ctrl.speed_x, chassis_ctrl.target_speed_x);
	PID_calc(&PID_LQR_k3, chassis_ctrl.angle_x, 0);
	PID_calc(&PID_LQR_k4, chassis_ctrl.gyro_x, 0);
	PID_calc(&PID_LQR_k5, chassis_ctrl.angle_z, 0);
//	if(chassis_ctrl.key_rc == RC_SW_MID)
//	{
//		PID_clear(&PID_LQR_k5);
//		PID_LQR_k5.out = 0;
//	}
		
	if(chassis_ctrl.wz_set!=0)
	{
		PID_clear(&PID_LQR_k1);
		PID_clear(&PID_LQR_k2);
		PID_clear(&PID_LQR_k5);
		PID_LQR_k3.Kp = K3;
		PID_LQR_k4.Kp = K4;		//ת��ʱ��λ�û����ٶȻ����������ǣ��ǶȻ��ͽ��ٶȻ�pֵ������
		PID_offset.Ki = 0.0;
		chassis_ctrl.target_pose_x=chassis_ctrl.pose_x;
		float speed_temp = chassis_ctrl.wz_set;	
//		PID_calc(&PID_LQR_k5, chassis_ctrl.INS_gyro[2], 6);
		PID_LQR_k5.out = ramp_control(chassis_ctrl.omega_z,speed_temp,0.25);
//		PID_LQR_k5.out	= speed_temp;
	}
	else
	{
		PID_LQR_k3.Kp = PID_LQR_k3_KP;		
		PID_LQR_k4.Kp = PID_LQR_k4_KP;
		PID_offset.Ki = PID_offset_KI;
	}
	if(fabs(chassis_ctrl.angle_x)>8.0/57.3)PID_LQR_k5.out *= Logistic(-fabs(chassis_ctrl.angle_x),20,-0.20f,1.0);//���ǿ��Ʋ�������ǰ����������Ҳ���Ǽ��ٶȹ���ʱ��������Ҫ������̨���Լ�����
	Chassis_RefereePowerControl(&PowerLimit_k5);
//	Chassis_CapPowerControl(&PowerLimit_k5);
	PID_calc(&PID_LQR_k6, chassis_ctrl.omega_z, PID_LQR_k5.out);
	
	chassis_ctrl.torque_speed = (-PID_LQR_k1.out - PID_LQR_k2.out) * chassis_ctrl.torque_const;
	chassis_ctrl.torque_balance = (- PID_LQR_k3.out - PID_LQR_k4.out) * chassis_ctrl.torque_const;
	chassis_ctrl.torque_omega = PID_LQR_k6.out * chassis_ctrl.torque_const;
	
	Chassis_RefereePowerControl(&PowerLimit_speed);
//	Chassis_CapPowerControl(&PowerLimit_speed);
	
	PID_calc(&PID_offset, 0, chassis_ctrl.torque_speed + chassis_ctrl.torque_balance);

	
	chassis_ctrl.GivenValue[0] = (-PID_offset.out+chassis_ctrl.torque_omega);
	chassis_ctrl.GivenValue[1] = (PID_offset.out+chassis_ctrl.torque_omega);
}

/**
  * @brief  ��������ģʽ
  */
void chassis_normal()
{
	PID_LQR_k1.Kp = 0.0f;//0.01f;
	PID_LQR_k2.Ki = 0.05f;
//	PID_LQR_k2.max_iout = 25.0f;//25.0f;
	chassis_ctrl.follow_gimbal_zero=CHASSIS_FOLLOW_GIMBAL_ANGLE_ZERO;
	chassis_ctrl.target_pose_x=chassis_ctrl.pose_x;
	chassis_ctrl.flag_clear_pose = 0;
	chassis_ctrl.target_speed_x=fabs(arm_cos_f32(PID_LQR_k5.error[0])) * speed_control(chassis_ctrl.move_speed,chassis_ctrl.move_direction);
	
	if(fabs(PID_LQR_k5.error[0])>0.22*PI)chassis_ctrl.target_speed_x=0;//�����������̨���ĽǶȹ��������ͣ����
	chassis_ctrl.target_speed_x=limit(chassis_ctrl.target_speed_x,-SPEED_UNLIMIT,SPEED_UNLIMIT);
	chassis_ctrl.angle_z=deadband_limit(angle_z_err_get(-chassis_ctrl.move_direction+(motor_gimbal.ecd / 1303.7973f)),0.02f);

	PID_calc(&PID_LQR_k1, chassis_ctrl.pose_x, chassis_ctrl.target_pose_x);
	PID_calc(&PID_LQR_k2, chassis_ctrl.speed_x, chassis_ctrl.target_speed_x);
	PID_calc(&PID_LQR_k3, chassis_ctrl.angle_x, 0);
	PID_calc(&PID_LQR_k4, chassis_ctrl.gyro_x, 0);
	PID_calc(&PID_LQR_k5, chassis_ctrl.angle_z, 0);
	
	if(fabs(chassis_ctrl.angle_x)>8.0/57.3)PID_LQR_k5.out *= Logistic(-fabs(chassis_ctrl.angle_x),20,-0.20f,1.0);//��б�ʹ���Ͳ�����������̨��,�����ֵͦ�õ���ʱ���øģ��Ĺ�����Ч���е���
	Chassis_RefereePowerControl(&PowerLimit_k5);
//	Chassis_CapPowerControl(&PowerLimit_k5);

	PID_calc(&PID_LQR_k6, chassis_ctrl.omega_z, PID_LQR_k5.out);
	
	
	chassis_ctrl.torque_speed = (-PID_LQR_k1.out - PID_LQR_k2.out) * chassis_ctrl.torque_const;
	chassis_ctrl.torque_balance = (- PID_LQR_k3.out - PID_LQR_k4.out) * chassis_ctrl.torque_const;
	chassis_ctrl.torque_omega = PID_LQR_k6.out * chassis_ctrl.torque_const;
	
	Chassis_RefereePowerControl(&PowerLimit_speed);
//	Chassis_CapPowerControl(&PowerLimit_speed);
		
	PID_calc(&PID_offset, 0, chassis_ctrl.torque_speed + chassis_ctrl.torque_balance);

	chassis_ctrl.GivenValue[0] = (-PID_offset.out+chassis_ctrl.torque_omega);
	chassis_ctrl.GivenValue[1] = (PID_offset.out+chassis_ctrl.torque_omega);
}

/**
  * @brief  ���̱���ģʽ
  * @param  ģʽ:0,ִ��һ��;1,�������˳�
  * @retval none
  */
void chassis_stop()
{
	chassis_state_clear();
	angle_z_err_get(motor_gimbal.ecd / 1303.7973f);
	float speed_x_real = chassis_ctrl.vx_set;
	if(chassis_ctrl.chassis_direction == CHASSIS_BACK)speed_x_real *= -1;
	PID_calc(&PID_MF9025_speed[0], motor_chassis[0].speed, -400*speed_x_real-300*chassis_ctrl.vy_set);
	PID_calc(&PID_MF9025_speed[1], motor_chassis[1].speed, 400*speed_x_real-300*chassis_ctrl.vy_set);
	
	chassis_ctrl.GivenValue[0] = PID_MF9025_speed[0].out;
	chassis_ctrl.GivenValue[1] = PID_MF9025_speed[1].out;
	if(Chassis_Capacitance<0.30)chassis_weak();
}

/**
  * @brief  ��������ģʽ
  * @param  none
  * @retval none
  */
void chassis_weak()
{
	chassis_state_clear();
	chassis_ctrl.GivenValue[0] = 0;
	chassis_ctrl.GivenValue[1] = 0;
}

/**
  * @brief  ����ƽ��ģʽ
  * @param  none
  * @retval none
  */
void chassis_balance()
{
	chassis_ctrl.pose_x = 0;
	chassis_ctrl.target_pose_x=0;
	chassis_ctrl.flag_clear_pose = 0;
	PID_clear(&PID_LQR_k1);
	PID_clear(&PID_LQR_k2);
	PID_calc(&PID_LQR_k3, chassis_ctrl.angle_x, 0);
	PID_calc(&PID_LQR_k4, chassis_ctrl.gyro_x, 0);
	PID_calc(&PID_LQR_k5, chassis_ctrl.angle_z, 0);
	Chassis_RefereePowerControl(&PowerLimit_k5);
	Chassis_CapPowerControl(&PowerLimit_k5);
	PID_calc(&PID_LQR_k6, chassis_ctrl.omega_z, PID_LQR_k5.out);
	chassis_ctrl.torque_balance = (- PID_LQR_k3.out - PID_LQR_k4.out) * chassis_ctrl.torque_const;
	chassis_ctrl.torque_omega = PID_LQR_k6.out * chassis_ctrl.torque_const;
	
	PID_calc(&PID_offset, 0, chassis_ctrl.torque_balance);

	chassis_ctrl.GivenValue[0] = (-PID_offset.out+chassis_ctrl.torque_omega);
	chassis_ctrl.GivenValue[1] = (PID_offset.out+chassis_ctrl.torque_omega);
}

/**
  * @brief  ���̴�ģʽ
  * @param  none
  * @retval none
  */
void chassis_slipped()
{
	chassis_ctrl.pose_x = 0;
	chassis_ctrl.target_pose_x=0;
	chassis_ctrl.flag_clear_pose = 0;
	PID_clear(&PID_LQR_k1);
	PID_clear(&PID_LQR_k2);
	PID_calc(&PID_LQR_k3, chassis_ctrl.angle_x, 0);
	PID_calc(&PID_LQR_k4, chassis_ctrl.gyro_x, 0);
	PID_clear(&PID_LQR_k5);
	Chassis_RefereePowerControl(&PowerLimit_k5);
	Chassis_CapPowerControl(&PowerLimit_k5);
	PID_calc(&PID_LQR_k6, chassis_ctrl.omega_z, PID_LQR_k5.out);
	chassis_ctrl.torque_balance = (- PID_LQR_k3.out - PID_LQR_k4.out) * chassis_ctrl.torque_const;
	chassis_ctrl.torque_omega = PID_LQR_k6.out * chassis_ctrl.torque_const;
	
	PID_calc(&PID_offset, 0, chassis_ctrl.torque_balance);

	chassis_ctrl.GivenValue[0] = (-PID_offset.out+chassis_ctrl.torque_omega);
	chassis_ctrl.GivenValue[1] = (PID_offset.out+chassis_ctrl.torque_omega);
}