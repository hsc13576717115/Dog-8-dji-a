#ifndef REMOCTRL_TASK_H
#define REMOCTRL_TASK_H

#include "serial.h"
#include "gait_param.h"
#include "cycloid_generator.h"
#include "Composite_Cycloid.h"

extern Robohorse_State Smart_State;

#define STEPLENTH_MAX 14.0f                     // 步长最大值
#define RC_CPLTCTRL_FREQ_MAX 4.0f               // 遥控器控制步频最大值
#define RC_CPLTCTRL_STEPLENTH_MAX 16.5f         // 遥控器控制步幅最大值  普通14.5 竞速16.5
#define RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX 4.0f // 遥控器微调步幅最大值
#define RC_CPLTCTRL_FLIGHTPERCENT_MAX 0.2f      // 遥控器控制摆动相占比最大值

#define RC_CPLTCTRL_STEPLENTH_turn_MAX 8.0f//16.0f			 // 遥控器控制步幅最大值 8.0f
#define RC_CPLTCTRL_STEPLENTH_climb_MAX 16.0f			 // 遥控器控制步幅最大值 8.0f
#define RC_CPLTCTRL_STEPLENTH_CH2_MAX 12.0f		 // CH2步幅调整最大值
#define RC_CPLTCTRL_STEPLENTH_CH2 16.0f		 // CH2步幅调整最大值

#define TX_DATAF32_SIZE (8) // 机器马发送数据数量(float32)
#define RX_DATA32_SIZE (1)  // 机器马接收数据数量(int32)

#define TX_DATA8_SIZE (TX_DATAF32_SIZE * 4) // 机器马发送数据数量(uint8)
#define RX_DATA8_SIZE (RX_DATA32_SIZE * 4)  // 机器马接受数据数量(uint8)

typedef struct
{
	float CH0_Smart;
	float CH1_Smart;
	float CH2_Smart;
	float CH3_Smart;
} Smart_CH_Smart;

extern Smart_CH_Smart CH;	  // 自主时的各个通道值 由手动调节 从而达到转弯和直行
extern uint8_t flag_imu_line; // imu直线修正标志
extern uint8_t creep_flag;

void RemoCtrl_Task(void *argument);

void Ctrl_byRemoctrl(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData);
void Change_RmVAL(Cycloid_Generator_t *pCycGenerato, RC_ctrl_t *pRCData, Robohorse_State state);
void Change_servo(uint8_t id, RC_ctrl_t *pRCData);

#endif /*REMOCTRL_TASK_H*/

