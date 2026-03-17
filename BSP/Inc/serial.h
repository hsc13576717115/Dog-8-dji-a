#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

typedef __packed struct
{
    __packed struct
    {
        int16_t ch[5];
        char s[2];
    } rc; // 遥控器数据
    __packed struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    __packed struct
    {
        uint16_t v;
    } key;
    uint8_t SW1_VALUE;
    uint8_t SW2_VALUE;
} RC_ctrl_t; // 航模遥控器控制结构体

typedef enum
{
    RC_NULL = 0,

    RC_11 = 0x11,
    RC_12,
    RC_13,
    RC_21 = 0x21,
    RC_22,
    RC_23,
    RC_31 = 0x31,
    RC_32,
    RC_33
} RC_Key; // 航模遥控器开关位置枚举

#define RC_SW_UP ((uint16_t)1)   // 开关上拨
#define RC_SW_MID ((uint16_t)3)  // 开关中拨
#define RC_SW_DOWN ((uint16_t)2) // 开关下拨

#define RC_CH_VALUE_MIN ((uint16_t)364)     // 手柄最小值
#define RC_CH_VALUE_OFFSET ((uint16_t)1024) // 手柄偏移值
#define RC_CH_VALUE_MAX ((uint16_t)16 // 手柄最小值

#define IF_RC_SW2_UP (rc_ctrl.rc.s[1] == RC_SW_UP)
#define IF_RC_SW2_MID (rc_ctrl.rc.s[1] == RC_SW_MID)
#define IF_RC_SW2_DOWN (rc_ctrl.rc.s[1] == RC_SW_DOWN)

#define IF_RC_SW1_UP (rc_ctrl.rc.s[0] == RC_SW_UP)
#define IF_RC_SW1_MID (rc_ctrl.rc.s[0] == RC_SW_MID)
#define IF_RC_SW1_DOWN (rc_ctrl.rc.s[0] == RC_SW_DOWN)

#define RC_CH0_RLR_OFFSET (rc_ctrl.rc.ch[0])
#define RC_CH1_RUD_OFFSET (rc_ctrl.rc.ch[1])
#define RC_CH2_LLR_OFFSET (rc_ctrl.rc.ch[2])
#define RC_CH3_LUD_OFFSET (rc_ctrl.rc.ch[3])
#define RC_CH4_SIDE_OFFSET (rc_ctrl.rc.ch[4])
/* ----------------------- RC Switch Definition----------------------------- */
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

extern uint8_t rc_data[18];
extern RC_ctrl_t my_rc_ctrl;
extern RC_Key rc_sw_state;

void RC_Start(void);
void RC_Transform(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#endif



