#ifndef __VMC_H__
#define __VMC_H__

// C点极坐标
typedef struct polarCoordinates_C
{
    float zeta;
    float L;
} polarCoordinates_C;

// 并联腿关节部分角度
typedef struct angle_leg
{
    float zeta1;
    float zeta2;
    float zeta3;
    float zeta4;
} angle_leg;

// B,C,D的xy轴坐标
typedef struct coordinate
{
    float x_B;
    float y_B;
    float x_C;
    float y_C;
    float x_D;
    float y_D;
} coordinate;

void get_angle(polarCoordinates_C *c, float F, float Fp);

// -------------------- 机械结构参数 --------------------
#define THGH_LNGTH 150.00 // 大腿长
#define CLF_LNGTH 270.00  // 小腿长

// -------------------- 梯形轨迹规划参数 --------------------
#define ACCLRTN 100.00f // 加速度
#define DCLRTN 1.00f    // 减速度
#define MXMMSPD 10.00f  // 最大速度

#endif
