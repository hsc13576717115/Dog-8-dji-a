#ifndef __ESP_H__
#define __ESP_H__

#include "stm32f4xx.h"

#define ch0val 1900.0f  //[-1800,1985] 归零后的范围 未归零的范围均为[0,4095] 
#define ch1val 1900.0f  //[-1800,1985]
#define ch2val 1900.0f  //[-1800,1985] 
#define ch3val 1900.0f  //[-1800,1985] 

#define ESP_MESSAGE_LENGTH 19
#define UP_MESSAGE_LENGTH  5
typedef struct
{
	uint8_t * data;
	uint8_t buf_size;
	struct
	{
		int16_t ch[5];
		uint8_t s[4];
	}rc;//遥控器数据
	
	
//	struct
//	{
//		uint8_t LL;
//		uint8_t LR;
//		uint8_t RL;
//		uint8_t RR;
//	}tog;//挡位
	
	struct
	{
		uint8_t forward;
		uint8_t back;
		uint8_t left;
		uint8_t right;
		uint8_t RF;
		uint8_t RB;
	}key;//按钮
	
	int8_t sw;//急停暂时不用
}ESP_ctrl_t; //ESP遥控器控制结构体


typedef enum
{
	ESP_NULL,
	ESP_111 = 0x111,
	ESP_112,
	ESP_113,
	ESP_121 = 0x121,
	ESP_122,
	ESP_123,
	ESP_131 = 0x131,
	ESP_132,
	ESP_133,
	
	ESP_211 = 0x211,
	ESP_212,
	ESP_213,
	ESP_221 = 0x221,
	ESP_222,
	ESP_223,
	ESP_231 = 0x231,
	ESP_232,
	ESP_233,
	
	ESP_311 = 0x311,
	ESP_312,
	ESP_313,
	ESP_321 = 0x321,
	ESP_322,
	ESP_323,
	ESP_331 = 0x331,
	ESP_332,
	ESP_333,
	
}ESP_KEY;//拨杆状态/模式

extern ESP_ctrl_t my_esp_ctrl;
extern uint8_t esp_data[19];
extern ESP_KEY esp_sw_state;
extern uint32_t CH1test;




void ESP_Init(void);
void ESP_Transform(uint8_t *ESP_data,ESP_ctrl_t * esp_ctrl);
void ESPCtrl_Data_Handle(ESP_ctrl_t *esp_ctrl);

void DOWM_TO_UP(uint8_t * data);
uint8_t crc8_calculate(uint8_t *data, uint8_t len);

#endif
