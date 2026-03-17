#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "stdint.h"
#include "string.h"
#include "usart.h"

#define Ultrasonic_Num 3

typedef struct 
{
	uint8_t frame_end;// ึกฮฒ
	uint8_t RxBuffer[Ultrasonic_Num];// สýพÝึก
	uint16_t distance;
}Usart_ultrasonic;

extern Usart_ultrasonic ultrasonic_Data;
extern __IO uint8_t ch3[Ultrasonic_Num];
extern long my_distance;

void USART_UltrasonicInit(Usart_ultrasonic *Data);
void Ultrasonic_ReadDate(uint8_t *buff);
void Ultrasonic_DealDate(void);

#endif
