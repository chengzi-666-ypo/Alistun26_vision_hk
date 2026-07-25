/*
 * cboard_uart_stm32.c
 *
 * 示例：基于 STM32 HAL 的中断接收实现（适用于 STM32Cube + HAL）
 * - 假设你已经配置了 UART（例如 huart1）并启用了接收中断或 DMA
 * - 本文件给出中断回调和简单的状态机处理逻辑
 *
 * 说明：具体的 HAL 初始化（MX_USARTx_UART_Init 等）和工程配置由你在 Keil/STM32CubeMX 中完成。
 * 你只需把以下代码片段合并到你的工程中并对接 uart 句柄与回调。
 */

#include "main.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define FRAME_START 0xAA
#define FRAME_END   0x55

extern UART_HandleTypeDef huart1; // 在你的工程中声明/定义

static bool use_crc = false; // 可由配置设置

// CRC8 与 POSIX 端实现相同
static uint8_t crc8(const uint8_t *data, size_t len, uint8_t poly)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x80) crc = ((crc << 1) ^ poly) & 0xFF;
            else crc = (crc << 1) & 0xFF;
        }
    }
    return crc & 0xFF;
}

// 缓冲与状态
#define RX_BUF_SIZE 256
static uint8_t rx_buf[RX_BUF_SIZE];
static volatile size_t rx_len = 0;
static uint8_t rx_byte;

// 请在初始化时启用接收中断：
// HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

// 当接收到一个字节时 HAL 会调用此回调（若使用 IT 模式）
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        // 将字节追加到缓冲区（注意并发访问）
        if (rx_len < RX_BUF_SIZE) rx_buf[rx_len++] = rx_byte;
        // 重新启动接收
        HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
    }
}

// 需要在主循环中调用解析函数
// LED 控制：根据 payload[0] 的命令控制 PH12/PH11/PH10
// 约定：payload[0] 的值表示命令：
//   0 -> 点亮 RED (PH12)，其他熄灭
//   1 -> 点亮 GREEN (PH11)，其他熄灭
//   2 -> 点亮 BLUE (PH10)，其他熄灭
//   3 -> 全部熄灭
// 说明：LED 为高电平点亮（连接方式为 source / 共阴或直接驱动），因此写 GPIO_PIN_SET 点亮。
static void set_leds_for_cmd(uint8_t cmd)
{
    // 定义引脚（STM32F427IGH6，GPIOH）
    // PH12 = LED_R, PH11 = LED_G, PH10 = LED_B
    switch (cmd) {
        case 0: // RED on
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
            break;
        case 1: // GREEN on
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
            break;
        case 2: // BLUE on
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);
            break;
        case 3: // all off
        default:
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);
            break;
    }
}

// API for integration
// 启动 UART 单字节接收（在 main 初始化后调用）
void cboard_uart_start(void)
{
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);
}

// 设置是否启用 CRC 校验
void cboard_uart_set_crc(bool en)
{
    use_crc = en;
}

static void handle_frame(uint8_t id, uint8_t payload[8])
{
    // 使用 payload[0] 作为简单命令标识
    uint8_t cmd = payload[0];
    set_leds_for_cmd(cmd);

    // 可选：回写确认帧（回显原帧），保持和 POSIX 端一致的格式
    uint8_t out[12];
    size_t idx = 0;
    out[idx++] = FRAME_START;
    out[idx++] = id;
    memcpy(&out[idx], payload, 8); idx += 8;
    if (use_crc) {
        uint8_t c = crc8((const uint8_t[]){id, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]}, 9, 0x07);
        out[idx++] = c;
    }
    out[idx++] = FRAME_END;
    HAL_UART_Transmit(&huart1, out, idx, HAL_MAX_DELAY);
}

// 在主循环中周期性调用，用来解析已有字节
void cboard_uart_process(void)
{
    size_t i = 0;
    while (i < rx_len) {
        if (rx_buf[i] != FRAME_START) { i++; continue; }
        size_t need = use_crc ? 12 : 10;
        if (i + need > rx_len) break; // 等待更多字节
        if (rx_buf[i + need - 1] != FRAME_END) { i++; continue; }
        uint8_t id = rx_buf[i+1];
        uint8_t payload[8];
        memcpy(payload, &rx_buf[i+2], 8);
        if (use_crc) {
            uint8_t rcrc = rx_buf[i+10];
            uint8_t calc = crc8((const uint8_t[]){id, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]}, 9, 0x07);
            if (calc != rcrc) {
                // CRC 错误，丢弃该起始字节
                i++;
                continue;
            }
        }
        handle_frame(id, payload);
        // 移动剩余字节
        size_t remain = rx_len - (i + need);
        memmove(rx_buf, rx_buf + i + need, remain);
        rx_len = remain;
        i = 0;
    }
}

/* 使用示例：
int main(void)
{
    HAL_Init();
    // 初始化 UART, clocks, peripherals ...

    // 启动单字节 IT 接收
    HAL_UART_Receive_IT(&huart1, &rx_byte, 1);

    while (1) {
        cboard_uart_process();
        // 其他任务
    }
}
*/
