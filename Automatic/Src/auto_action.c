#include "auto_action.h"
#include "posture_task.h"

#include "jy901s.h"
#include "pid.h"

Robohorse_State Smart_State = WALK; // 自主行走模式设定

//前进
void AUTO_advance(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2)
{
    CH.CH3_Smart = CH3;
		if(CH2 == 0.0f)
			CH.CH2_Smart = 0.0f;
		else if(CH2 > 0.0f)
			CH.CH2_Smart = 0.50f;
		else if(CH2 < 0.0f)
			CH.CH2_Smart = -0.50f;
    
    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE)
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart+ RC_CPLTCTRL_STEPLENTH_turn_MAX * (CH.CH2_Smart > 0 ? CH.CH2_Smart : 0);
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart+ RC_CPLTCTRL_STEPLENTH_turn_MAX * (CH.CH2_Smart > 0 ? CH.CH2_Smart : 0);
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart- RC_CPLTCTRL_STEPLENTH_turn_MAX * (CH.CH2_Smart > 0 ? 0 : CH.CH2_Smart);
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart- RC_CPLTCTRL_STEPLENTH_turn_MAX * (CH.CH2_Smart > 0 ? 0 : CH.CH2_Smart);
    }
    else
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
    }
    Change_NowState(state);
	Calculate_Gait(Smart_State);
	Target_toMotor();
}

//旋转
void AUTO_turn(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH2)
{
	CH.CH2_Smart = CH2 > 0 ? 0.8f : -0.8f;

    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE)
    {
        pCycGenerato[0].param.step_length =    RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[1].param.step_length =    RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[2].param.step_length =  - RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[3].param.step_length =  - RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
    }
    else
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ( CH.CH0_Smart);
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ( CH.CH0_Smart);
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (-CH.CH0_Smart);
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (-CH.CH0_Smart);
    }
    Change_NowState(state);
	Calculate_Gait(Smart_State);
	Target_toMotor();
}

void AUTO_turn_PID(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2)
{
	CH.CH3_Smart = CH3;  
	PID_Calc(&PID1_AUTO, 0, CH2);
	CH.CH2_Smart = PID1_AUTO.pos_out;

    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE)
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart + RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart + RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart - RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart - RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
    }
    else
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ( CH.CH0_Smart);
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ( CH.CH0_Smart);
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (-CH.CH0_Smart);
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (-CH.CH0_Smart);
    }
    Change_NowState(state);
	Calculate_Gait(Smart_State);
	Target_toMotor();
}


















//直接在原来的上面改有点抽象，所以另起一个
void AUTO_JY901s_advance(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2,float zheng,float fu)
{
	//试一下CH2写死看下怎么走的 以及增加左右的一步的距离 或者是0.1f? 或者试一下原地转
		//CH2>0 顺时针转 <0 逆时针转
    CH.CH3_Smart = CH3;  //[-2,2]
		CH.CH2_Smart = CH2;
	
	
    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE || state == CREEP || state == RUN)
    {
        pCycGenerato[0].param.step_length = pCycGenerato[0].param.change_step_length * (CH.CH3_Smart + CH.CH2_Smart);
        pCycGenerato[1].param.step_length = pCycGenerato[0].param.change_step_length * (CH.CH3_Smart + CH.CH2_Smart);
        pCycGenerato[2].param.step_length = pCycGenerato[0].param.change_step_length * (CH.CH3_Smart - CH.CH2_Smart);
        pCycGenerato[3].param.step_length = pCycGenerato[0].param.change_step_length * (CH.CH3_Smart - CH.CH2_Smart);
			
    }
    else
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
    }
    Change_NowState(state);
	Calculate_Gait(state);
	Target_toMotor();
}
















void AUTO_vision_advance(Cycloid_Generator_t *pCycGenerato, Robohorse_State state, float CH3, float CH2)
{
	CH.CH3_Smart = CH3;  //[-2,2]
		if (-1.0f<CH2 && CH2<1.0f) CH2=0.0f;
		if(CH2 == 0.0f)
			CH.CH2_Smart = 0.0f;
		else if(CH2 > 0.0f)
			CH.CH2_Smart = 0.052;  //0.083
		else if(CH2 < -0.0f)
			CH.CH2_Smart = -0.052f;
    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE || state == CREEP)
    {
        pCycGenerato[0].param.step_length = pCycGenerato[0].param.change_step_length * CH.CH3_Smart+ pCycGenerato[0].param.change_step_length/2 * (CH.CH2_Smart > 0 ? CH.CH2_Smart : 0);
        pCycGenerato[1].param.step_length = pCycGenerato[0].param.change_step_length * CH.CH3_Smart+ pCycGenerato[0].param.change_step_length/2 * (CH.CH2_Smart > 0 ? CH.CH2_Smart : 0);
        pCycGenerato[2].param.step_length = pCycGenerato[0].param.change_step_length * CH.CH3_Smart- pCycGenerato[0].param.change_step_length/2 * (CH.CH2_Smart > 0 ? 0 : CH.CH2_Smart);
        pCycGenerato[3].param.step_length = pCycGenerato[0].param.change_step_length * CH.CH3_Smart- pCycGenerato[0].param.change_step_length/2 * (CH.CH2_Smart > 0 ? 0 : CH.CH2_Smart);
			
    }
    else
    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
    }
    Change_NowState(state);
	Calculate_Gait(Smart_State);
	Target_toMotor();
}

