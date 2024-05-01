#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H
#include "main.h"
#include "can.h"
#include "Filter.h"

#define KEY_PRESSED_OFFSET_W            ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S            ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A            ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D            ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT        ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL         ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q            ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E            ((uint16_t)1 << 7)

#define KEY_PRESSED_OFFSET_R            ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F            ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G            ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z            ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X            ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C            ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V            ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B            ((uint16_t)1 << 15)

typedef enum
{
	CAN_CHASSIS_ALL_ID = 0x200,
	CAN_3508_M1_ID = 0x201,
	CAN_3508_M2_ID = 0x202,
	
	CAN_MF9025_ALL_ID = 0x140,
	CAN_MF9025_1_ID = 0x141,
	CAN_MF9025_2_ID = 0x142,
	
	CAN_6020_M1_ID = 0x206,
	
	CAN_CAP_TX_ID = 0x140,
	CAN2_SHOOT_JUDGE_ID = 0x020C,
	
	CAN2_CONNECT_RC_CTRL_STD_ID = 0x200,
} can_msg_id_e;

//电机反馈结构体
typedef struct
{
    uint16_t ecd;
    int16_t speed;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
		lpf_data_t LPF_speed_rpm;
} motor_measure_t;

typedef struct   //can2传输的rc数据
{
	uint8_t control_mode;
	uint8_t work_mode;
	struct 
	{
		int16_t ch2;
		int16_t ch3;
	}rc;
	struct 
	{
		float yaw_set;
		float yaw_fdb;
	}gyro;
	struct
	{
		int16_t key;
	}mouse;

}can2_rc_ctrl_t;

typedef struct
{
	//RC_ctrl_t *rc_ctrl;
	
	can2_rc_ctrl_t can2_rc_ctrl;
	uint8_t receive_success_flag;
	uint8_t receive_rc_data_flag;
}connect_t;

//向底盘电机发送
void CAN_cmd_chassis(int16_t left,int16_t right);
//向超级电容发送
void CAN_Cap_CMD(float data1,float data2,float data3,float data4);
//向上板发送int16_t*4
void CAN_Gimbal_CMD(uint16_t Id ,int16_t data1, int16_t data2, int16_t data3, int16_t data4);
//向上板发送float*2
void CAN_Gimbal_CMD_fp32(uint16_t Id ,float data1, float data2);
//处理上板消息
void data_rx_handler(void);

extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern motor_measure_t motor_gimbal;
extern motor_measure_t motor_chassis[2];
extern int16_t Can_Data_RX16[];
extern uint8_t Can_Data_RX8[];
extern float debug_data_fp32[];


//超级电容		
extern float Chassis_CapPower;			//底盘总功率
extern float Chassis_Capacitance;		//剩余容量
extern uint8_t UI_Capacitance;   		//电容剩余容量

extern float   UI_Gimbal_Pitch;
extern uint8_t autoaim_mode;
extern uint8_t autoaim_armor;
extern uint8_t if_predict;
extern uint8_t UI_fric_is_on;    //摩擦轮是否开启

#endif

