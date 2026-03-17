/**
 * @file app_btdebug 蓝牙调试
 * @author Zhiyuan Mao(2019th)
 * @note 基于手机 蓝牙调试器 APP专业调试模式设计，全平台通用
 */
#ifndef __APP_BTDEBUG_H
#define __APP_BTDEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stdint.h"

#define BTDEBUG_TRUE 0
#define BTDEBUG_FALSE 1

    /**
     * @brief 蓝牙调试操作结构体
     */
    typedef struct
    {
        /**
         * @brief 发送字节调用函数
         * @param pData (uint8_t *) 数据指针
         * @param Size (uint32_t) 数据大小
         */
        void (*TransmitBytes)(uint8_t *pData, uint32_t Size);
        /**
         * @brief 接收字节调用函数
         * @param pData (uint8_t *) 数据指针
         * @param Size (uint32_t) 数据大小
         */
        void (*ReceiveBytes)(uint8_t *pData, uint32_t Size); //
    } BTDebug_Operate_t;

    /**
     * @brief 蓝牙调试发送数据包
     * @param opt (BTDebug_Operate_t *)蓝牙调试操作结构体
     * @param pData (uint8_t *)数据指针
     * @param Size (uint32_t)数据大小
     * @return NULL
     */
    void BTDebug_TransmitPackage(BTDebug_Operate_t *opt, uint8_t *pData, uint32_t Size);

    /**
     * @brief 蓝牙调试接受数据包
     * @param opt (BTDebug_Operate_t *)蓝牙调试操作结构体
     * @param pData (uint8_t *)数据指针
     * @param Size (uint32_t)数据大小
     * @return BTDEBUG_TRUE(0) or BTDEBUG_FALSE(1)
     */
    uint8_t BTDebug_ReceivePackage(BTDebug_Operate_t *opt, uint8_t *pData, uint32_t Size);

#ifdef __cplusplus
}
#endif

#endif /*__APP_BTDEBUG_H*/

