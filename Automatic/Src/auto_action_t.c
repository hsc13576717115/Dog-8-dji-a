#include "auto_action_t.h"
#include "posture_task.h"

#include "jy901s.h"
#include "pid.h"

void AUTO_advance_t(Composite_Cycloid_t *pCycGenerato, Robohorse_State state, float CH3, float CH2)
{
    CH.CH3_Smart = CH3;
	
//    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE)
//    {
        pCycGenerato[0].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart + RC_CPLTCTRL_STEPLENTH_CH2_MAX * (CH.CH2_Smart > 0 ? 0.5f : 0);
        pCycGenerato[1].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart + RC_CPLTCTRL_STEPLENTH_CH2_MAX * (CH.CH2_Smart > 0 ? 0.5f : 0);
        pCycGenerato[2].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart - RC_CPLTCTRL_STEPLENTH_CH2_MAX * (CH.CH2_Smart > 0 ? 0 : -0.5f);
        pCycGenerato[3].param.step_length = RC_CPLTCTRL_STEPLENTH_MAX * CH.CH3_Smart - RC_CPLTCTRL_STEPLENTH_CH2_MAX * (CH.CH2_Smart > 0 ? 0 : -0.5f);
//    }
//    else
//    {
//        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
//        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
//        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
//        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * CH.CH1_Smart;
//    }
    Change_NowState(state);
	Calculate_Gait(Smart_State);
	Target_toMotor();
}

void AUTO_turn_t(Composite_Cycloid_t *pCycGenerato, Robohorse_State state, float CH2)
{
	CH.CH2_Smart = CH2;

//    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE)
//    {
		pCycGenerato[0].param.step_length =   RC_CPLTCTRL_STEPLENTH_CH2 * CH.CH2_Smart;//(CH.CH2_Smart > 0 ? 0 : CH.CH2_Smart);
        pCycGenerato[1].param.step_length =   RC_CPLTCTRL_STEPLENTH_CH2 * CH.CH2_Smart;//(CH.CH2_Smart > 0 ? 0 : CH.CH2_Smart);
        pCycGenerato[2].param.step_length = - RC_CPLTCTRL_STEPLENTH_CH2 * CH.CH2_Smart;//(CH.CH2_Smart > 0 ? CH.CH2_Smart : 0);
        pCycGenerato[3].param.step_length = - RC_CPLTCTRL_STEPLENTH_CH2 * CH.CH2_Smart;//(CH.CH2_Smart > 0 ? CH.CH2_Smart : 0);
//    }
//    else
//    {
//        pCycGenerato[0].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ( CH.CH0_Smart);
//        pCycGenerato[1].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * ( CH.CH0_Smart);
//        pCycGenerato[2].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (-CH.CH0_Smart);
//        pCycGenerato[3].param.step_length = RC_CPLTCTRL_FINETUNE_STEPLENTH_MAX * (-CH.CH0_Smart);
//    }
    Change_NowState(state);
	Calculate_Gait(Smart_State);
	Target_toMotor();
}

void AUTO_turn_PID_t(Composite_Cycloid_t *pCycGenerato, Robohorse_State state, float CH2)
{
	PID_Calc(&PID_AUTO, 0, CH2);
	CH.CH2_Smart = PID_AUTO.pos_out;

    if (state == TROT || state == WALK || state == CLIMBING || state == MINCE)
    {
        pCycGenerato[0].param.step_length =   RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[1].param.step_length =   RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[2].param.step_length = - RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
        pCycGenerato[3].param.step_length = - RC_CPLTCTRL_STEPLENTH_CH2_MAX * CH.CH2_Smart;
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

void AUTO_turn_Gyro_t(Composite_Cycloid_t *pCycGenerato, float set_angle)
{
	float now_z_data = Now_Angle.yaw;

    float AUTO_fix;
    float err = set_angle - now_z_data;
    err = err > 180.0f ? (-180.0f + fmodf(err, 180.0f)) : (err < -180) ? (180 + fmodf(err, -180.0f))
                                                                       : err;
    PID_Calc(&PID_AUTO, err, 0);
    AUTO_fix = PID_AUTO.pos_out;
    AUTO_turn_t(pCycGenerato, Smart_State, AUTO_fix);
    Calculate_Gait(Smart_State); // 自主行走选择的步态
    Target_toMotor();            // 目标坐标转化角度并输出到电机
}
