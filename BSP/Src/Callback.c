#include "pid.h"
#include "jy901s.h"
#include "Vision.h"
#include "Laser.h"
#include "serial.h"
#include "ultrasonic.h"
#include "esp.h"

#include "stdlib.h"

#include "usart.h"


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
    {
        /* 必须停止DMA才能重新装载需要读取的字符数“18” */
        HAL_UART_DMAStop(&huart1);
        /* 设置传输数据长度*/
        HAL_UART_Receive_DMA(&huart1, (uint8_t *)rc_data, 18);
    }
	else if (huart == &huart4)
	{
		HAL_UART_DMAStop(&huart4);
	    Laser_DecodeDate(&Laser1);
		HAL_UART_Receive_DMA(&huart4, (uint8_t *)ch4, 11);
	}
	else if (huart == &huart7)
    {
		HAL_UART_DMAStop(&huart7);
		/* 校验摄像头数据 */
		Vision_ReadDate(vision_Data.RxBuffer);
		/* 处理摄像头数据 */
		Vision_DealDate();
        HAL_UART_Receive_DMA(&huart7, (uint8_t *)ch7, Vision_Num);
	}
//	else if (huart == &huart7)
//    {
//		HAL_UART_DMAStop(&huart7);
//		/* 校验摄像头数据 */
//		Ultrasonic_ReadDate(ultrasonic_Data.RxBuffer);
//		/* 处理摄像头数据 */
//		Ultrasonic_DealDate();
//        HAL_UART_Receive_DMA(&huart7, (uint8_t *)ch7, Ultrasonic_Num);
//	}
    else if (huart == &huart8)
    {
        HAL_UART_DMAStop(&huart8);
        /* 处理陀螺仪数据 */
        CopeSerial2Data((uint8_t*)ch8);
			
				/* 处理ESP遥控器数据 */
				
        /* 设置传输数据长度 */
        HAL_UART_Receive_DMA(&huart8, (uint8_t*)ch8, 11);
    }
}
