#include "jy901s.h"
#include <string.h>
#include <stdint.h>
#include "usart.h"
#include "Ahrs_Task.h"

uint8_t jy901_on = 1;

__IO uint8_t ch8[11]; // UART8陀螺仪数据缓存区

__IO double AttiAcc[3] = {0};
__IO double AttiGyro[3] = {0};
__IO double AttiAngle[3] = {0};

uint8_t ACCCALSW[5] = {0XFF, 0XAA, 0X01, 0X01, 0X00};	 // 进入加速度校准模式
uint8_t ZRETURNZERO[5] = {0XFF, 0XAA, 0X01, 0X04, 0X00}; // Z轴归零
uint8_t SAVACALSW[5] = {0XFF, 0XAA, 0X00, 0X00, 0X00};	 // 保存当前配置
uint8_t SETBAUDRATE[5] = {0XFF, 0XAA, 0X04, 0X06, 0X00}; // 设置波特率为115200
uint8_t SET_AXIS_6[5] = {0XFF, 0XAA, 0X24, 0X01, 0X00};	 // 设置为6轴算法
uint8_t SET_AXIS_9[5] = {0XFF, 0XAA, 0X24, 0X00, 0X00};	 // 设置为9轴算法
uint8_t SETUNLOCK[5] = {0XFF, 0XAA, 0X69, 0X88, 0XB5};	 // 解锁指令
uint8_t SETRSW[5] = {0XFF, 0XAA, 0X02, 0X08, 0X00};		 // 设置回传内容
uint8_t SETRATE[5] = {0XFF, 0XAA, 0X03, 0X09, 0X00};	 // 设置回传速率100hz

extern int _test4;

void JY901_Init()
{
	UART8_Init();
	HAL_Delay(100);
	JY901_sendcmd(SETUNLOCK);
	HAL_Delay(150); // 解锁指令 100ms后发送其他指令 10秒后失效
	JY901_sendcmd(SET_AXIS_6);
	HAL_Delay(150); // 设置6轴算法
	JY901_sendcmd(SAVACALSW);
	HAL_Delay(150); // 保存当前配置
	JY901_sendcmd(ACCCALSW);
	HAL_Delay(150); // 等待模块内部自动校准好，模块内部会自动计算需要一定的时间
	JY901_sendcmd(SAVACALSW);
	HAL_Delay(150); // 保存当前配置
	JY901_sendcmd(ZRETURNZERO);
	HAL_Delay(150); // Z轴归零
	JY901_sendcmd(SETRSW);
	HAL_Delay(150);
	JY901_sendcmd(SETRATE);
	HAL_Delay(150);
	JY901_sendcmd(SAVACALSW);
	HAL_Delay(1000); // 保存当前配置

}
// 读取JY901数据
void JY901_Read(AHRS_Angle_t *Now_Angle)
{
	Now_Angle->roll = AttiAngle[0];
	Now_Angle->pitch = AttiAngle[1];
	Now_Angle->yaw = AttiAngle[2];
}

// 用串口8给JY模块发送指令
void JY901_sendcmd(uint8_t *cmd)
{
	HAL_UART_Transmit_DMA(&huart8, cmd, 5);
}

// CopeSerialData为串口2中断调用函数，串口每收到一个数据，调用一次这个函数。
void CopeSerial2Data(uint8_t *ucData)
{
	uint8_t ucCnt = 0;
	uint8_t ucDataBuf[JY901_FRAME_LENGTH] = {0};

	while (ucData[ucCnt] != 0x55 && ucCnt < JY901_FRAME_LENGTH) // 寻找数据帧头
	{
		ucCnt++;
	}

	if (ucCnt>=JY901_FRAME_LENGTH) return;
	memcpy(ucDataBuf, &ucData[ucCnt], 11);

	if (ucCnt != 0)
	{
		uint8_t i = 0;
		for (i = 0; i < ucCnt; i++)
			ucDataBuf[JY901_FRAME_LENGTH - ucCnt + i] = ucData[i];
	}

	if (ucDataBuf[0] == 0x55)
	{
		struct SAcc stcAcc;  
		struct SGyro stcGyro;
		struct SAngle stcAngle;

		switch (ucDataBuf[1]) // 判断数据是哪种数据，然后将其拷贝到对应的结构体中，有些数据包需要通过上位机打开对应的输出后，才能接收到这个数据包的数据
		{
		case 0x51    :
			memcpy(&stcAcc, &ucDataBuf[2], 8);
			AttiAcc[0] = (double)stcAcc.a[0] / 32768.0 * 16 * 9.8;
			AttiAcc[1] = (double)stcAcc.a[1] / 32768.0 * 16 * 9.8;
			AttiAcc[2] = (double)stcAcc.a[2] / 32768.0 * 16 * 9.8;
			break;
		case 0x52:

			memcpy(&stcGyro, &ucDataBuf[2], 8);
			AttiGyro[0] = (double)stcGyro.w[0] * 2000.0 / 32768.0;
			AttiGyro[1] = (double)stcGyro.w[1] * 2000.0 / 32768.0;
			AttiGyro[2] = (double)stcGyro.w[2] * 2000.0 / 32768.0;
			break;
		case 0x53:
			memcpy(&stcAngle, &ucDataBuf[2], 8);
			AttiAngle[0] = (double)stcAngle.Angle[0] * 180 / 32768.0;
			AttiAngle[1] = (double)stcAngle.Angle[1] * 180 / 32768.0;
			AttiAngle[2] = (double)stcAngle.Angle[2] * 180 / 32768.0;

			break;
		}
	}
}
