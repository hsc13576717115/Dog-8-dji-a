/**
 * 航模遥控器及蓝牙控制
 * 大疆的航模遥控器暂时无法驱动成功
 * 蓝牙控制使用手机上 蓝牙调试器 APP进行调试
 */
#include "serial.h"
#include "app_ebf_piddebug.h"
#include "ultrasonic.h"
#include "usart.h"
#include "jy901s.h"
#include "Vision.h"
#include "Laser.h"
#include "stdlib.h"

extern __IO uint8_t ch8[11]; // UART8陀螺仪数据缓存区

uint8_t rc_data[18];
RC_ctrl_t my_rc_ctrl;
RC_Key rc_sw_state = RC_NULL;


void RemoCtrl_Data_Handle(RC_ctrl_t *rc_ctrl);

/*启动航模遥控器*/
void RC_Start()
{
    HAL_UART_Receive_DMA(&huart1, rc_data, 18);
}

void RC_Transform(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = ((int16_t)sbus_buf[0] | (int16_t)(sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((int16_t)(sbus_buf[1] >> 3) | (int16_t)(sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((int16_t)(sbus_buf[2] >> 6) | (int16_t)(sbus_buf[3] << 2) |          //!< Channel 2
                         (int16_t)(sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((int16_t)(sbus_buf[4] >> 1) | (int16_t)(sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                                    //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x0003);                                         //!< Switch right
    rc_ctrl->mouse.x = (int16_t)sbus_buf[6] | ((int16_t)sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = (int16_t)sbus_buf[8] | (int16_t)(sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = (int16_t)sbus_buf[10] | (int16_t)(sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                                    //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                                    //!< Mouse Right Is Press ?
    rc_ctrl->key.v = (int16_t)sbus_buf[14] | (int16_t)(sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = (int16_t)sbus_buf[16] | (int16_t)(sbus_buf[17] << 8);                 // NULL

    RemoCtrl_Data_Handle(rc_ctrl);
}
/**
 * @brief  遥控器数据处理及状态更新
 * @param rc_ctrl 遥控器数据
 */
void RemoCtrl_Data_Handle(RC_ctrl_t *rc_ctrl)
{
    /* 更新遥控开关状态 */
    rc_sw_state = (RC_Key)((rc_ctrl->rc.s[0] << 4) | (rc_ctrl->rc.s[1]));
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//    if (huart == &huart1)
//    {
//        /* 必须停止DMA才能重新装载需要读取的字符数“18” */
//        HAL_UART_DMAStop(&huart1);
//        /* 设置传输数据长度*/
//        HAL_UART_Receive_DMA(&huart1, (uint8_t *)rc_data, 18);
//    }
//    else if (huart == &huart7)
//    {
//	HAL_UART_DMAStop(&huart7);
//		/* 校验摄像头数据 */
//		Ultrasonic_ReadDate(ultrasonic_Data.RxBuffer);
//		/* 处理摄像头数据 */
//		Ultrasonic_DealDate();
//        HAL_UART_Receive_DMA(&huart7, (uint8_t *)ch7, Ultrasonic_Num);
//    }
//    else if (huart == &huart8)
//    {
//        HAL_UART_DMAStop(&huart8);
//        /* 处理陀螺仪数据 */
//        CopeSerial2Data((uint8_t *)ch8);
//        /* 设置传输数据长度 */
//        HAL_UART_Receive_DMA(&huart8, (uint8_t *)ch8, 11);
//    }

//}
