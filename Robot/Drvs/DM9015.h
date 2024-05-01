#ifndef __DM9015_H
#define __DM9015_H

#include "main.h"
#include "usart.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "SlideBlock_Task.h"

#define DM9015_RX_BUF_NUM 8

typedef struct{
	uint16_t Angle;
	uint16_t LastAngle;
	int64_t TotalAngle;
	int16_t OffsetAngle;
	int16_t SetAngle;
	int64_t RoundCnt;
	uint8_t Calibration;
	int16_t CalibratedAngle;
	int16_t CalibratedLastAngle;
}DM9015_measure_t;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern uint8_t DM9015_rx_buf[2][DM9015_RX_BUF_NUM];
extern DM9015_measure_t DM9015_measure[2];

void DM9015_usart_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void DM9015_unpack(uint8_t* data);
void TotalAngleUpdate(DM9015_measure_t *ptr);
void TotalAngleCalibrate(DM9015_measure_t *ptr);
void DM9015_send(int power,uint8_t id);


#endif
