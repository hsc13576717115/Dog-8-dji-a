#ifndef MY_USART_H
#define MY_USART_H

#include "stdint.h"
#include "stdio.h"
#include "usart.h"

#define ReBuff 11
typedef struct _Laser_Data_t
{
    uint8_t *data;
    uint8_t data_size;
    uint8_t frame_size;
    float temp;
} Laser_Data_t;

extern Laser_Data_t Laser1;
extern __IO uint8_t ch4[11];

void Laser_Process_Init(Laser_Data_t *Usart, uint8_t *data_buff, uint8_t size_buff, uint8_t frame_size);
void Laser_DecodeDate(Laser_Data_t *Usart);
uint8_t CalculateChecksum(uint8_t *data, uint8_t length);
void Laser_SendMessage(uint8_t *string);

#endif
