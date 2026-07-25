/*
 * cboard_uart.h
 *
 * 简单头文件，便于在 Keil/CubeMX 工程中集成 cboard_uart_stm32.c
 */

#ifndef CBOARD_UART_H
#define CBOARD_UART_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 在 main 初始化完成后调用以启动中断接收
void cboard_uart_start(void);

// 设置是否启用 CRC-8 校验（默认关闭）
void cboard_uart_set_crc(bool en);

// 在主循环中周期性调用以处理收到的帧
void cboard_uart_process(void);

#ifdef __cplusplus
}
#endif

#endif // CBOARD_UART_H
