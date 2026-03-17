#include "usart.h"
#include "motor_control.h"
#include "crc_ccitt.h"
#include "stdio.h"
#include "freeRTOS.h"
#include "task.h"

#define SATURATE(_IN, _MIN, _MAX) \
  {                               \
    if (_IN < _MIN)               \
      _IN = _MIN;                 \
    else if (_IN > _MAX)          \
      _IN = _MAX;                 \
  }

extern UART_HandleTypeDef huart6;

int modify_data(MOTOR_send *motor_s)  //发送数据转换
{
  motor_s->hex_len = 17;
  motor_s->motor_send_data.head[0] = 0xFE;
  motor_s->motor_send_data.head[1] = 0xEE;

  //		SATURATE(motor_s->id,   0,    15);
  //		SATURATE(motor_s->mode, 0,    7);
  SATURATE(motor_s->K_P, 0.0f, 25.599f);
  SATURATE(motor_s->K_W, 0.0f, 25.599f);
  SATURATE(motor_s->T, -127.99f, 127.99f);
  SATURATE(motor_s->W, -804.00f, 804.00f);
  SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

  motor_s->motor_send_data.mode.id = motor_s->id;
  motor_s->motor_send_data.mode.status = motor_s->mode;
  motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768; //就等于*1280
  motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768;
  motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.2832f * 32768;
  motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.2832f * 256;
  motor_s->motor_send_data.comd.tor_des = motor_s->T * 256;
  motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, 15);
  return 0;
}

int extract_data(MOTOR_recv *motor_r) //接收数据转换
{
  if (motor_r->motor_recv_data.CRC16 !=
      crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, 14))
  {
    // printf("[WARNING] Receive data CRC error");
    motor_r->correct = 0;
    return motor_r->correct;
  }
  else
  {
    motor_r->motor_id = motor_r->motor_recv_data.mode.id;
    motor_r->mode = motor_r->motor_recv_data.mode.status;
    motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
    motor_r->MError = motor_r->motor_recv_data.fbk.MError;
    motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256) * 6.2832f;
    motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256;
    motor_r->Pos = 6.2832f * ((float)motor_r->motor_recv_data.fbk.pos) / 32768;
    motor_r->footForce = motor_r->motor_recv_data.fbk.force;
    motor_r->correct = 1;
    return motor_r->correct;
  }
}

HAL_StatusTypeDef SERVO_Send_recv(MOTOR_send *pData, MOTOR_recv *rData)
{
  uint16_t rxlen = 0;

  modify_data(pData);

  SET_485_DE_UP();
  SET_485_RE_UP();
  HAL_UART_Transmit(&huart6, (uint8_t *)pData, sizeof(pData->motor_send_data), 20);

  SET_485_RE_DOWN();
  SET_485_DE_DOWN();
	taskENTER_CRITICAL();
  HAL_UARTEx_ReceiveToIdle(&huart6, (uint8_t *)rData, sizeof(rData->motor_recv_data), &rxlen, 10); //接收的一种 当10ms没有数据时跳出/数据收满了跳出
  taskEXIT_CRITICAL();
  extract_data(rData);  //数据转换

  if (rxlen == 0)

    return HAL_TIMEOUT;

  if (rxlen != sizeof(rData->motor_recv_data))
    return HAL_ERROR;

  uint8_t *rp = (uint8_t *)&rData->motor_recv_data;
  if (rp[0] == 0xFE && rp[1] == 0xEE)
  {
    rData->correct = 1;
    extract_data(rData);
    return HAL_OK;
  }

  return HAL_ERROR;
}


