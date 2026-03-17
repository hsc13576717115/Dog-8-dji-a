#include "app_btdebug.h"

/*全没用*/

void BTDebug_TransmitPackage(BTDebug_Operate_t *opt, uint8_t *pData, uint32_t Size)
{
    uint8_t tx_head = 0xa5;             // 协议头
    uint8_t tx_tail = 0x5a;             // 协议尾
    uint8_t corr = 0;                   // 校验码
    for (uint32_t i = 0; i < Size; i++) // 计算校验码，累加数据后取后8位
        corr += pData[i];
    opt->TransmitBytes(&tx_head, 1); // 发送协议头
    opt->TransmitBytes(pData, Size); // 发送数据
    opt->TransmitBytes(&corr, 1);    // 发送校验码
    opt->TransmitBytes(&tx_tail, 1); // 发送协议尾
}

uint8_t BTDebug_ReceivePackage(BTDebug_Operate_t *opt, uint8_t *pData, uint32_t Size)
{
    uint8_t rx_buf; // 接收缓冲
    uint8_t temp;
    do
    {
        opt->ReceiveBytes(&rx_buf, 1);
    } while (rx_buf != 0xa5);       // 轮询直到获得协议头
    opt->ReceiveBytes(pData, Size); // 读取数据
    opt->ReceiveBytes(&rx_buf, 1);  // 读取校验
    opt->ReceiveBytes(&temp, 1);    // 读取协议尾
    if (temp != 0x5a)               // 如果协议尾错误
    {
        return BTDEBUG_FALSE; // 返回错误状态码
    }
    temp = 0;
    for (uint32_t i = 0; i < Size; i++) // 计算校验码
        temp += pData[i];
    if (temp != rx_buf) // 若校验码错误
    {
        return BTDEBUG_FALSE; // 返回错误状态码
    }
    return BTDEBUG_TRUE;
}
