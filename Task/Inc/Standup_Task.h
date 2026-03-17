#ifndef __STANDUP_H__
#define __STANDUP_H__

#include <stdint.h>
#include "posture_task.h"
#include "serial.h"

#define PI 3.1415926

typedef enum
{
	LRNULL = 0,
	L,
	R
}CREEP_Leg_t;

extern float Standup_LegR_Offset;
extern float Standup_LegL_Offset;
extern float Standup_LegR_Offset_Right;
extern float Standup_LegL_Offset_Right;
extern float LegLinit_onestep;
extern float LegRinit_onestep;
extern float LegLinit_twostep;
extern float LegRinit_twostep;
extern float CREEP_LegR;
extern float CREEP_LegL;
extern CREEP_Leg_t flag_creep_LR;
extern uint8_t flag_Standup;

void Standup_Task(void *agrument);
void Setdown(void);
void CHANGE_HIGH(Robohorse_State now,RC_ctrl_t * rc_data,uint8_t * flag);
void CHANGE_KPW(float k_pt,float k_wt);
void CHANGE_HIGH_AUTO(Robohorse_State now,uint8_t flag_mode,uint8_t * flag);



#endif


