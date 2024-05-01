#include "DM9015.h"


uint8_t DM9015_rx_buf[2][DM9015_RX_BUF_NUM];
DM9015_measure_t DM9015_measure[2];

void DM9015_usart_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }
    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    hdma_usart1_rx.Instance->NDTR = 2*dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);
}


void USART1_IRQHandler_1(void)
{
	if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if(USART1->SR & UART_FLAG_IDLE)
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 2*DM9015_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = 2*DM9015_RX_BUF_NUM;
            //设定缓冲区1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == DM9015_RX_BUF_NUM)
            {
								DM9015_unpack(DM9015_rx_buf[0]);
            }
        }
        else
        {
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = 2*DM9015_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;
            //重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = 2*DM9015_RX_BUF_NUM;
            //设定缓冲区0
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if(this_time_rx_len == DM9015_RX_BUF_NUM)
            {
								DM9015_unpack(DM9015_rx_buf[1]);
            }
        }
    }
}

void DM9015_unpack(uint8_t* data)
{
	uint8_t crc = data[0] + data[1] + data[2] + data[3];
	if(data[4] != crc)return;
	crc = data[5] + data[6];
	if(data[7] != crc)return;
	
	if(data[2] == 0x01)
	{
		BaseType_t HigherPriorityTaskWoken=pdTRUE;
		xTaskNotifyFromISR(SlideBlock_TaskHandle,1,eSetValueWithOverwrite,&HigherPriorityTaskWoken);
		DM9015_measure[0].LastAngle = DM9015_measure[0].Angle;
		DM9015_measure[0].Angle = data[6]<<8|data[5];
		TotalAngleUpdate(&DM9015_measure[0]);
	}
	else if(data[2] == 0x02)
	{
		BaseType_t HigherPriorityTaskWoken=pdTRUE;
		xTaskNotifyFromISR(SlideBlock_TaskHandle,2 ,eSetValueWithOverwrite,&HigherPriorityTaskWoken);
		DM9015_measure[1].LastAngle = DM9015_measure[1].Angle;
		DM9015_measure[1].Angle = data[6]<<8|data[5];
		TotalAngleUpdate(&DM9015_measure[1]);
	}
}

void TotalAngleUpdate(DM9015_measure_t *ptr)
{
	if(ptr->Calibration == 0)
	{
		ptr->LastAngle = ptr->OffsetAngle;
		ptr->Calibration = 1;
	}
	if(ptr->Angle - ptr->LastAngle >  2048) ptr->RoundCnt--;
	if(ptr->Angle - ptr->LastAngle < -2048) ptr->RoundCnt++;
	ptr->TotalAngle = ptr->RoundCnt * (int64_t)4096 + ptr->Angle - ptr->OffsetAngle;

}

void DM9015_send(int power,uint8_t id)
{
	uint8_t ctrl_cmd[8];

	if(power>850) power=850;
	else if(power<-850) power=-850;
	ctrl_cmd[0]=0x3E; //frame head.
	ctrl_cmd[1]=0xA0; //cmd.
	ctrl_cmd[2]=id; 	//ID.
	ctrl_cmd[3]=0x02; //data length.
	ctrl_cmd[4]=ctrl_cmd[0]+ctrl_cmd[1]+ctrl_cmd[2]+ctrl_cmd[3]; //sum of [0]to[3]
	ctrl_cmd[5]=power;
	ctrl_cmd[6]=power >> 8;
	ctrl_cmd[7]=ctrl_cmd[5]+ctrl_cmd[6];
	HAL_UART_Transmit(&huart1,ctrl_cmd,sizeof(ctrl_cmd),10);
}