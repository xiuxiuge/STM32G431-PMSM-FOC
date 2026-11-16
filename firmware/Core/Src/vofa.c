/*
 * vofa.c
 *
 *  Created on: 2026年11月1日
 *      Author: 15332
 */
#include "vofa.h"
#include <string.h>

extern UART_HandleTypeDef hlpuart1;

// 8个float + 4字节JustFloat帧尾
static uint8_t tx_buffer[36];

// DMA发送状态
static volatile uint8_t tx_busy = 0;

// 记录因发送忙碌或启动失败而丢弃的帧数
volatile uint32_t vofa_drop_count = 0;


uint8_t VOFA_Send8(const float *data)
{
    // 检查指针
    if (data == NULL)
        return 0;

    // 上一帧尚未发送完成
    if (tx_busy)
    {
        vofa_drop_count++;
        return 0;
    }

    // 复制8个float，共32字节
    memcpy(tx_buffer, data, 8 * sizeof(float));

    // JustFloat帧尾
    tx_buffer[32] = 0x00;
    tx_buffer[33] = 0x00;
    tx_buffer[34] = 0x80;
    tx_buffer[35] = 0x7F;

    tx_busy = 1;

    // DMA异步发送
    if (HAL_UART_Transmit_DMA(&hlpuart1, tx_buffer, sizeof(tx_buffer)) != HAL_OK)
    {
        tx_busy = 0;
        vofa_drop_count++;
        return 0;
    }

    return 1;
}


// UART发送完成回调
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        tx_busy = 0;
    }
}


// UART发送错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        tx_busy = 0;
    }
}
