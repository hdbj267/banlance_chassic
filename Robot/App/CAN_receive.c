/**
  **********************************2022 CKYF***********************************
  * @file    CAN_receive.c
  * @brief   处理CAN收到的数据
  ******************************************************************************
	* @team    长空御风
	* @author  杨洛
  ******************************************************************************
  * @attention
  *	
  **********************************2022 CKYF***********************************
  */
	
#include "CAN_receive.h"
#include "Filter.h"
#include "Vofa_send.h"
#include "referee_usart_task.h"
#include "ChassisControl_Task.h"

extern CAN_HandleTypeDef hcan1;

#define RC_CHANNEL_VALUE_MIDDLE         (1024u) 

#define motor_measure_M3508(ptr, data)																	\
    {																																		\
        (ptr)->last_ecd = (ptr)->ecd;																		\
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);						\
        (ptr)->speed = (uint16_t)((data)[2] << 8 | (data)[3]);					\
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);	\
        (ptr)->temperate = (data)[6];																		\
    }

#define motor_measure_MF9025(ptr, data)																	\
    {																																		\
        (ptr)->last_ecd = (ptr)->ecd;																		\
        (ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);						\
        (ptr)->speed = -(int16_t)((data)[5] << 8 | (data)[4]);					\
        (ptr)->given_current = -(int16_t)((data)[3] << 8 | (data)[2]);	\
        (ptr)->temperate = (data)[1];																		\
    }
		
#define get_can_data_16(ptr, data)															\
    {																														\
        (ptr)[0] = (uint16_t)((data)[0] << 8 | (data)[1]);			\
        (ptr)[1] = (uint16_t)((data)[2] << 8 | (data)[3]);			\
        (ptr)[2] = (uint16_t)((data)[4] << 8 | (data)[5]);			\
        (ptr)[3] = (uint16_t)((data)[6] << 8 | (data)[7]);			\
    }

#define get_can_data_8(ptr, data)						\
    {																				\
        (ptr)[0] = (uint8_t)(data)[0]; 			\
        (ptr)[1] = (uint8_t)(data)[1]; 			\
        (ptr)[2] = (uint8_t)(data)[2]; 			\
        (ptr)[3] = (uint8_t)(data)[3]; 			\
        (ptr)[4] = (uint8_t)(data)[4]; 			\
			  (ptr)[5] = (uint8_t)(data)[5]; 			\
			  (ptr)[6] = (uint8_t)(data)[6]; 			\
			  (ptr)[7] = (uint8_t)(data)[7]; 			\
    }
	
static CAN_TxHeaderTypeDef motor_tx_message,cap_tx_message,gimbal_tx_message;
static uint8_t motor_can_send_data[8];
static uint8_t cap_can_send_data[8];
	
motor_measure_t motor_chassis[2];
motor_measure_t motor_gimbal;
connect_t connect_data;

//int16_t*4
int16_t Can_Data_RX16[4];
//uint8_t*8
uint8_t Can_Data_RX8[8];
//float*4
float debug_data_fp32[4];

//超级电容		
float Chassis_CapPower;			//底盘总功率
float Chassis_Capacitance = 1;	//剩余容量

void connect_rc_ctrl_process(connect_t *connect_data, uint8_t aData[])
{
	connect_data->can2_rc_ctrl.control_mode = (aData[0]);
	connect_data->can2_rc_ctrl.work_mode = (aData[1]);
	connect_data->can2_rc_ctrl.rc.ch3 = (aData[2]<<8)|aData[3];
	connect_data->can2_rc_ctrl.rc.ch2 = (aData[4]<<8)|aData[5];
	connect_data->can2_rc_ctrl.mouse.key = (aData[6]<<8)|aData[7];
	
	connect_data->can2_rc_ctrl.rc.ch2 -= RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch3 -= RC_CHANNEL_VALUE_MIDDLE;
	connect_data->can2_rc_ctrl.rc.ch2 = -connect_data->can2_rc_ctrl.rc.ch2;
	connect_data->receive_rc_data_flag = 1;//表示已经接收到了can2的rc数据
}

/**
 * @brief          hal库CAN回调函数,接收电机数据
 * @param[in]      hcan:CAN句柄指针
 * @retval         none
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	CAN_RxHeaderTypeDef rx_header;
	uint8_t rx_data[8];
	HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
	
	//CAN1
	if(hcan==&hcan1)
	{
		switch (rx_header.StdId)
		{
			//MF9025
			case CAN_MF9025_1_ID:
			case CAN_MF9025_2_ID:
			{
					static uint8_t i = 0;
					// get motor id
					i = rx_header.StdId - CAN_MF9025_1_ID;
					motor_measure_MF9025(&motor_chassis[i], rx_data);
					break;
			}
			//超级电容
//			case CAN_CAP_RX_ID:
//			{
//					Chassis_Capacitance = (float)((uint16_t)rx_data[1] << 8 | rx_data[0]) / 32768.0f;
//					Chassis_CapPower    = (float)((uint16_t)rx_data[3] << 8 | rx_data[2]) / 100.0f;
//					UI_Capacitance = Chassis_Capacitance*100;
//					break;
//			}
			default:
			{
					break;
			}
		}
	}
	
	//CAN2
	if(hcan==&hcan2)
	{
		switch (rx_header.StdId)
		{
			//YAW云台电机
			case CAN_6020_M1_ID:
			{
					motor_measure_M3508(&motor_gimbal, rx_data);
					break;
			}
			//上板消息
			case CAN2_CONNECT_RC_CTRL_STD_ID:
			{
					connect_rc_ctrl_process(&connect_data ,rx_data);
					break;
			}
			case 0x151:
			{
					get_can_data_8(Can_Data_RX8, rx_data);
					break;
			}
			default:
			{
					break;
			}
		}
		if(rx_header.StdId == 0x150 || rx_header.StdId == 0x151)
			data_rx_handler();
	}
}

//向底盘电机发送
// RMD9025
//void CAN_cmd_chassis(int16_t left, int16_t right)
//{
//     uint32_t send_mail_box;
//     motor_tx_message.StdId = 0x280;
//     motor_tx_message.IDE = CAN_ID_STD;
//     motor_tx_message.RTR = CAN_RTR_DATA;
//     motor_tx_message.DLC = 0x08;
//     motor_can_send_data[0] = left;
//     motor_can_send_data[1] = left >> 8;
//     motor_can_send_data[2] = right;
//     motor_can_send_data[3] = right >> 8;
//     motor_can_send_data[4] = 0x00;
//     motor_can_send_data[5] = 0x00;
//     motor_can_send_data[6] = 0x00;
//     motor_can_send_data[7] = 0x00;

//     HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
// }

//LK9025
void CAN_cmd_chassis(int16_t left, int16_t right)
{
     uint32_t send_mail_box;
     motor_tx_message.StdId = 0x280;
     motor_tx_message.IDE = CAN_ID_STD;
     motor_tx_message.RTR = CAN_RTR_DATA;
     motor_tx_message.DLC = 0x08;
     motor_can_send_data[0] = (-left);
     motor_can_send_data[1] = (-left) >> 8;
     motor_can_send_data[2] = (-right);
     motor_can_send_data[3] = (-right) >> 8;
     motor_can_send_data[4] = 0x00;
     motor_can_send_data[5] = 0x00;
     motor_can_send_data[6] = 0x00;
     motor_can_send_data[7] = 0x00;

     HAL_CAN_AddTxMessage(&hcan1, &motor_tx_message, motor_can_send_data, &send_mail_box);
 }

//向超级电容发送
void CAN_Cap_CMD(float data1,float data2,float data3,float data4)
{
	uint32_t send_mail_box;
	cap_tx_message.StdId = CAN_CAP_TX_ID;
	cap_tx_message.IDE = CAN_ID_STD;
	cap_tx_message.RTR = CAN_RTR_DATA;
	cap_tx_message.DLC = 0x08;
	
	uint16_t temp;
	temp=data1*100;
	cap_can_send_data[0] = temp;
	cap_can_send_data[1] = temp>> 8;
	temp=data2*100;
	cap_can_send_data[2] = temp;
	cap_can_send_data[3] = temp>> 8;
	temp=data3*100;
	cap_can_send_data[4] = temp;
	cap_can_send_data[5] = temp>> 8;
	temp=data4*100;
	cap_can_send_data[6] = temp;
	cap_can_send_data[7] = temp>> 8;

	HAL_CAN_AddTxMessage(&hcan1, &cap_tx_message, cap_can_send_data, &send_mail_box);
}

//向上板发送int16_t*4
void CAN_Gimbal_CMD(uint16_t Id ,int16_t data1, int16_t data2, int16_t data3, int16_t data4)
{
	uint32_t send_mail_box;
	uint8_t gimbal_can_send_data[8];		
	gimbal_tx_message.StdId = Id;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	gimbal_can_send_data[0] = data1 >> 8;
	gimbal_can_send_data[1] = data1;
	gimbal_can_send_data[2] = data2 >> 8;
	gimbal_can_send_data[3] = data2;
	gimbal_can_send_data[4] = data3 >> 8;
	gimbal_can_send_data[5] = data3;
	gimbal_can_send_data[6] = data4 >> 8;
	gimbal_can_send_data[7] = data4;

	HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//向上板发送float*2
void CAN_Gimbal_CMD_fp32(uint16_t Id ,float data1, float data2)
{
	uint32_t send_mail_box;
	uint8_t gimbal_can_send_data[8];		
	gimbal_tx_message.StdId = Id;
	gimbal_tx_message.IDE = CAN_ID_STD;
	gimbal_tx_message.RTR = CAN_RTR_DATA;
	gimbal_tx_message.DLC = 0x08;
	memcpy(gimbal_can_send_data+0	,&data1,4);
	memcpy(gimbal_can_send_data+4	,&data2,4);
	HAL_CAN_AddTxMessage(&hcan2, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

//获取底盘电机指针
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
    return &motor_chassis[(i & 0x01)];
}

//处理上板消息
void data_rx_handler()
{
	chassis_ctrl.vx_rc = deadband_limit(Can_Data_RX16[0],50);
	chassis_ctrl.vy_rc = deadband_limit(Can_Data_RX16[1],50);
	chassis_ctrl.wz_rc = deadband_limit(Can_Data_RX16[2],50);
	chassis_ctrl.key_rc = Can_Data_RX8[0];

	UI_Gimbal_Pitch = Can_Data_RX16[3];
	autoaim_mode = Can_Data_RX8[1];
	autoaim_armor = Can_Data_RX8[2];
	if_predict = Can_Data_RX8[3];
	UI_fric_is_on = Can_Data_RX8[4];
}