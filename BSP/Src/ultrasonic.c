#include "ultrasonic.h"
#include "math.h"


/*È«Ã»ÓÃ*/

Usart_ultrasonic ultrasonic_Data;

__IO uint8_t ch3[Ultrasonic_Num];
long my_distance = 0;

void USART_UltrasonicInit(Usart_ultrasonic *Data)
{
	Data->frame_end = 0;
	for (uint8_t i = 0; i < Ultrasonic_Num; i++)
	{
		Data->RxBuffer[i] = 0;
	}
}
	
void Ultrasonic_ReadDate(uint8_t *buff)
{
	uint8_t sum = 0;
	for(uint8_t i = 0; i < Ultrasonic_Num - 1; i++)
	{
		sum += ch3[i];
	}
	sum &= 0xFF;
	if(sum != ch3[Ultrasonic_Num - 1])
		return;
	memcpy(buff, (uint8_t*)ch3, Ultrasonic_Num);
}
	
void Ultrasonic_DealDate(void)
{
	my_distance = 0;
	ultrasonic_Data.distance = (ultrasonic_Data.RxBuffer[0] << 8) + ultrasonic_Data.RxBuffer[1];
	for(uint8_t i = 0; i < 4; i++)
	{
		my_distance += ((ultrasonic_Data.distance >> 4 * i) & 0xF) * pow(16, i);
	}
	
	memset(ultrasonic_Data.RxBuffer, 0, 3);
}
