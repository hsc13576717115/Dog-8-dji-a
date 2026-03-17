#include "laser.h"
#include "string.h"
#include <stdlib.h>
#include "stdint.h"

Laser_Data_t Laser1;
__IO uint8_t ch4[11];

float last_temp = 0;

uint8_t laser_bufsingle[4] = {0x80, 0x06, 0x02, 0x78};
uint8_t laser_bufcoiled[4] = {0x80, 0x06, 0x03, 0x77};

extern UART_HandleTypeDef huart4;

void Laser_Process_Init(Laser_Data_t *Usart, uint8_t *data_buff, uint8_t size_buff, uint8_t frame_size)
{

	Usart->data = data_buff;
	Usart->data_size = size_buff;
	Usart->frame_size = frame_size;

	HAL_UART_Transmit_DMA(&huart4, (uint8_t *)laser_bufcoiled, sizeof(laser_bufcoiled));
}

void Laser_SendMessage(uint8_t *string)
{
	HAL_UART_Transmit_DMA(&huart4, (uint8_t *)string, sizeof(string));
}

void Laser_DecodeDate(Laser_Data_t *Usart)
{
	uint8_t *ucDataBuf = (uint8_t *)malloc(Usart->frame_size);
	uint8_t ucCnt = 0;
	const uint8_t *ucData = Usart->data;

	while (ucData[ucCnt] != 0x80) // 寻找数据帧头
	{
		ucCnt++;
		if (ucCnt == ReBuff)
		{
			free(ucDataBuf);
			return;
		}
	}

	memcpy(ucDataBuf, &ucData[ucCnt], Usart->frame_size - ucCnt);

	if (ucCnt != 0)
	{
		uint8_t i = 0;
		for (i = 0; i < ucCnt; i++)
			ucDataBuf[Usart->frame_size - ucCnt + i] = ucData[i];
	}

	/* У��ʹ��� */
	if (ucDataBuf[10] != CalculateChecksum(ucDataBuf, 10)) //校验 全加取反加1
	{
		free(ucDataBuf);
		return;
	}

	Usart->temp = (ucDataBuf[3] - 48) * 100000 + (ucDataBuf[4] - 48) * 10000 + (ucDataBuf[5] - 48) * 1000 + (ucDataBuf[7] - 48) * 100 + (ucDataBuf[8] - 48) * 10 + (ucDataBuf[9] - 48) * 1;
	if(Usart->temp > 10000.0f || Usart->temp < 0)
	{
		Usart->temp = last_temp;
	}
	last_temp = Usart->temp;
	free(ucDataBuf);
	return;
}

uint8_t CalculateChecksum(uint8_t *data, uint8_t length)
{
	uint8_t checksum = 0;
	for (uint8_t i = 0; i < length; i++)
	{
		checksum += data[i];
	}
	return ~(checksum) + 1;
}
