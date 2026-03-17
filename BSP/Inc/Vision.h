#ifndef __VISION_H__
#define __VISION_H__

#include "stdint.h"
#include "string.h"
#include "usart.h"

#define Vision_Num 6

enum Move_t
{
	stop = 1,
	run,
	back,
	left,
	right
};

typedef struct 
{
	uint8_t frame_head;// ึกอท
	uint8_t frame_end;// ึกฮฒ
	uint8_t RxBuffer[Vision_Num];// สýพÝึก
	float acting_already;// 
	float x_error;
	float dis;
	float fb_speed;
	float lr_speed;
	uint8_t vision_state;
	int action;
}Usart_vision;

extern enum Move_t action;
extern Usart_vision vision_Data;
extern __IO uint8_t ch7[Vision_Num];

void USART_VisionInit(Usart_vision *Data);
void Vision_ReadDate(uint8_t *buff);
void Vision_DealDate(void);

#endif
