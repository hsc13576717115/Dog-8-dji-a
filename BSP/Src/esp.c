#include "esp.h"

#include "usart.h"

#include "string.h"

ESP_ctrl_t my_esp_ctrl;
uint8_t esp_data[19]={0};
ESP_KEY esp_sw_state=ESP_NULL;
uint8_t test111 = 0;
//uint32_t CH1test=0;
//uint8_t initia=1;

//启动
void ESP_Init(void)
{
	HAL_UART_DMAStop(&huart8);
	
	//HAL_UART_Receive_DMA(&huart8,esp_data,ESP_MESSAGE_LENGTH );
}

void ESP_Transform(uint8_t *ESP_data,ESP_ctrl_t * esp_ctrl)
{
	if (ESP_data==NULL)
	{
		return ;
	}
	uint8_t ucCnt=0;
	uint8_t ucData[ESP_MESSAGE_LENGTH]={0};
	uint8_t ucDataBuf[ESP_MESSAGE_LENGTH]={0};
	memcpy(ucData,ESP_data,ESP_MESSAGE_LENGTH);
	
	while(ucData[ucCnt]!=0xFF && ucCnt<ESP_MESSAGE_LENGTH)
	{
		ucCnt++;
	}
	if (ucCnt>=ESP_MESSAGE_LENGTH) return;
	memcpy(ucDataBuf,&ucData[ucCnt],ESP_MESSAGE_LENGTH - ucCnt);
	for(int i=0;i<ucCnt;i++)
	{
		ucDataBuf[ESP_MESSAGE_LENGTH - ucCnt + i] = ucData[i];
	}
	test111 = crc8_calculate(ucDataBuf,18);
	if (ucDataBuf[18] != crc8_calculate(ucDataBuf,18))
	{
		return ;  //数据错误 不读取
	}	
	
	esp_ctrl->rc.ch[0] = ((ucDataBuf[1]<<8)|ucDataBuf[2]);//右x
	esp_ctrl->rc.ch[1] = ((ucDataBuf[3]<<8)|ucDataBuf[4]);//右y减值暂定
	esp_ctrl->rc.ch[2] = ((ucDataBuf[5]<<8)|ucDataBuf[6]);//左x
	esp_ctrl->rc.ch[3] = ((ucDataBuf[7]<<8)|ucDataBuf[8]);//左y减值暂定
	
//	if (esp_ctrl->rc.ch[0]>1920 && esp_ctrl->rc.ch[0]<1940) esp_ctrl->rc.ch[0]=ch0val;
//	if (esp_ctrl->rc.ch[1]>1770 && esp_ctrl->rc.ch[1]<1800) esp_ctrl->rc.ch[1]=ch1val;
//	if (esp_ctrl->rc.ch[2]>1930 && esp_ctrl->rc.ch[2]<1960) esp_ctrl->rc.ch[2]=ch2val;
//	if (esp_ctrl->rc.ch[3]>1770 && esp_ctrl->rc.ch[3]<1800) esp_ctrl->rc.ch[3]=ch3val;
	
	//esp中已进行死区 为防止数值跳跃 故限制
	for (int i=0;i<4;i++)
	{
		if (esp_ctrl->rc.ch[i] > 1900) esp_ctrl->rc.ch[i] -= 2110;
		else if (esp_ctrl->rc.ch[i] == 1900) esp_ctrl->rc.ch[i] -= ch0val;
		else if (esp_ctrl->rc.ch[i] < 1900) esp_ctrl->rc.ch[i] -= 1800;
	}
//	esp_ctrl->rc.ch[0]-=ch0val;
//	esp_ctrl->rc.ch[1]-=ch1val;
//	
//	esp_ctrl->rc.ch[2]-=ch2val;
//	esp_ctrl->rc.ch[3]-=ch3val;
		
	esp_ctrl->rc.s[0]= (ucDataBuf[9]<<4)  | ucDataBuf[10];
	esp_ctrl->rc.s[1]=  ucDataBuf[11];
	
	esp_ctrl->key.forward	= (ucDataBuf[12]);
	esp_ctrl->key.back 		= (ucDataBuf[13]);
	esp_ctrl->key.left 		= (ucDataBuf[14]);
	esp_ctrl->key.right 	= (ucDataBuf[15]);
	esp_ctrl->key.RF			= (ucDataBuf[16]);
	esp_ctrl->key.RB			= (ucDataBuf[17]);
	
	ESPCtrl_Data_Handle(esp_ctrl);
	if (esp_ctrl->rc.ch[0] < -1900 || esp_ctrl->rc.ch[0] > 2200) esp_ctrl->rc.ch[0] = 0;
	if (esp_ctrl->rc.ch[2] < -1900 || esp_ctrl->rc.ch[2] > 2200) esp_ctrl->rc.ch[2] = 0;
	if (esp_ctrl->rc.ch[1] < -1900 || esp_ctrl->rc.ch[1] > 2200) esp_ctrl->rc.ch[1] = 0;
	if (esp_ctrl->rc.ch[3] < -1900 || esp_ctrl->rc.ch[3] > 2200) esp_ctrl->rc.ch[3] = 0;
}

void ESPCtrl_Data_Handle(ESP_ctrl_t *esp_ctrl)
{
	esp_sw_state = (ESP_KEY)((esp_ctrl->rc.s[0]<<4) | (esp_ctrl->rc.s[1]));
}

/* CRC校验 */
uint8_t crc8_calculate(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
		{
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
				{
            if (crc & 0x80)
						{
                crc = (crc << 1) ^ 0x07;
            } 
						else
						{
                crc <<= 1;
            }
        }
    }
    return crc;
}


