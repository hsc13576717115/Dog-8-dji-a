#include "Vision.h"
#include "math.h"

enum Move_t action;
Usart_vision vision_Data;

__IO uint8_t ch7[Vision_Num];

float now_time_t = 0;
float prev_time_t = 0;

float prev_distance = 0;

void USART_VisionInit(Usart_vision *Data)
{
	Data->frame_head = 0xFF;
	Data->frame_end = 0;
	Data->x_error = 0;
	Data->dis = 0;
	Data->action = 1;
	for (uint8_t i = 0; i < Vision_Num; i++)
	{
		Data->RxBuffer[i] = 0;
	}
}

void Vision_ReadDate(uint8_t *buff)
{
	uint8_t ucCnt = 0;
	uint8_t ucData[Vision_Num] = {0};
	uint8_t ucDataBuf[Vision_Num] = {0};
	memcpy(ucData,(uint8_t *)ch7,Vision_Num);
	
	while(ucData[ucCnt]!=0xFF && ucCnt<Vision_Num)
	{
		ucCnt++;
	}
	if (ucCnt>=Vision_Num) return;
	memcpy(ucDataBuf,&ucData[ucCnt],Vision_Num-ucCnt);
	for(int i=0;i<ucCnt;i++)
	{
		ucDataBuf[Vision_Num - ucCnt + i] = ucData[i];
	}
	
	
	
	vision_Data.fb_speed = ucDataBuf[1] / 99.0f;
	if (ucDataBuf[2]) vision_Data.fb_speed*=-1.0f;
	vision_Data.lr_speed = ucDataBuf[3] / 99.0f;
	if (ucDataBuf[4]) vision_Data.lr_speed*=-1.0f;
	vision_Data.vision_state = ucDataBuf[5];
	
	
	memset(vision_Data.RxBuffer, 0, Vision_Num);
	
	
	
	
	
//	
//	if (ch7[0] != 0xFF)
//		return;
//	uint8_t sum = 0;
//	for(uint8_t i = 0; i < Vision_Num - 1; i++)
//	{
//		sum += ch7[i];
//	}
//	if(sum != ch7[Vision_Num - 1])
//		return;
	
	
	memcpy(buff, ucDataBuf, Vision_Num);
}

void Vision_DealDate(void)
{ 
	now_time_t = HAL_GetTick() / 1000.0f; //获取当前时间用于后续运算
	switch(vision_Data.RxBuffer[1])
	{
	case 0xAA:
		if(vision_Data.RxBuffer[2] == 0x00)
		{
			vision_Data.x_error = -((int)(vision_Data.RxBuffer[3]) + (int)(vision_Data.RxBuffer[4]) * 0.1);
		}
		else if(vision_Data.RxBuffer[2] == 0x01)
		{
			vision_Data.x_error = ((int)(vision_Data.RxBuffer[3]) + (int)(vision_Data.RxBuffer[4]) * 0.1);
		}
		
		vision_Data.action = (int)vision_Data.RxBuffer[5];
		
		//vision_Data.dis = ((int)(vision_Data.RxBuffer[6]) + (int)(vision_Data.RxBuffer[7]) * 0.1);
		break;
	}
	memset(vision_Data.RxBuffer, 0, Vision_Num);
}

