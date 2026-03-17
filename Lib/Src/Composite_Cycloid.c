#include "Composite_Cycloid.h"
#include <math.h>
#include "pid.h"
#include "gait_param.h"

#include "stdio.h"
/*Ã»ÓÃ*/
#define N_PI 3.1415926

//void Composite_Cycloid_Calc(
//    float now_time,
//    Composite_Cycloid_t *cyc,
//    float phase,
//    Composite_Cycloid_Solution_t *solution)
//{
//    static float p = 0.0f;
//    static float prev_t = 0.0f;

//	float stanceHeight = cyc->param.body_height;
//    float stepLenght = cyc->param.step_length;
//    float upAMP = cyc->param.up_amp;
//	float flightPercent = cyc->param.flight_percent;
//    float FREQ = cyc->param.freq;

//    p += FREQ * (now_time - prev_t < 0.5f ? now_time - prev_t : 0);
//    prev_t = now_time;

//    float gp = fmod((p + phase + flightPercent / 2), 1.0f);
//    if (gp < flightPercent)
//    {
//        solution->x = stepLenght * (gp / flightPercent - sin(2 * N_PI * gp / flightPercent) / (2 * N_PI)) - stepLenght / 2;
//        if(0 <= gp && gp < flightPercent / 2)
//			solution->y = stanceHeight - 2 * upAMP * (gp / flightPercent - sin(4 * N_PI * gp / flightPercent) / (4 * N_PI));
//		else if(flightPercent / 2 <= gp && gp < flightPercent)
//			solution->y = stanceHeight - 2 * upAMP * (1 - gp / flightPercent + sin(4 * N_PI * gp / flightPercent) / (4 * N_PI));
//    }
//    else
//    {
//        solution->x = stepLenght / 2 - stepLenght * ((gp - flightPercent) / (1 - flightPercent) + sin(2 * N_PI * gp / (1 - flightPercent)) / (2 * N_PI));
//        solution->y = stanceHeight;
//    }
//}
