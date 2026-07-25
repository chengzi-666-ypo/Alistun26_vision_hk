/*
 * cboard_uart_posix.c
 *
 * 示例：在基于 Linux 的 C-board 或带 POSIX UART 的设备上运行的串口接收程序
 * 协议（Protocol C）: 0xAA | id(1) | payload(8) | [crc?] | 0x55
 * - 可选 CRC-8 (poly 0x07, init 0x00) 放在 payload 后
 *
 * 功能：
 * - 打开串口设备（使用 termios）
 * - 持续读取字节，按状态机解析完整帧
 * - 校验可选 CRC（如果启用）
 * - 调用 handle_frame(id, payload, payload_len)
 * - 示例的 handle_frame 会把收到的帧原样回写（Echo）
 *
 * 编译：
 *   gcc -o cboard_uart_posix cboard_uart_posix.c
 * 运行：
 *   sudo ./cboard_uart_posix /dev/ttyACM0 115200 [--crc]
 *
 * 注意：如果运行在普通 Linux（非 root），请保证当前用户有访问串口的权限。
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>

#define FRAME_START 0xAA
#define FRAME_END   0x55

static bool use_crc = false;

// CRC-8 (MSB-first) 与 Python 实现兼容（poly 0x07, init 0x00）
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

// 将完整帧原样写回（回显）
static void send_frame(int fd, uint8_t id, const uint8_t payload[8])
{
    // 构建帧
    uint8_t buf[12]; // 最大: 0xAA id payload(8) crc 0x55 => 11 bytes; we keep 12 for safety
    int idx = 0;
    buf[idx++] = FRAME_START;
    buf[idx++] = id;
    memcpy(&buf[idx], payload, 8); idx += 8;
    if (use_crc) {
        uint8_t c = crc8((const uint8_t[]){id, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]}, 9, 0x07);
        buf[idx++] = c;
    }
    buf[idx++] = FRAME_END;

    ssize_t w = write(fd, buf, idx);
    (void)w; // ignore write result in example
}

// 处理已解析 payload 的回调（可替换为真正的业务逻辑）
static void handle_frame(int fd, uint8_t id, const uint8_t payload[8])
{
    printf("[CBoard] handle_frame id=0x%02x payload=", id);
    for (int i = 0; i < 8; ++i) printf(" %02x", payload[i]);
    printf("\n");

    // 示例：原样回写
    send_frame(fd, id, payload);
}

// 配置串口（8N1，无流控），返回 fd 或 -1
static int open_serial(const char *dev, int baud)
{
    int fd = open(dev, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("open");
        return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("tcgetattr");
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);

    // 设置波特率
    speed_t speed;
    switch (baud) {
        case 115200: speed = B115200; break;
        case 57600: speed = B57600; break;
        case 38400: speed = B38400; break;
        case 19200: speed = B19200; break;
        case 9600: speed = B9600; break;
        default: speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS; // no flow control
    tty.c_cflag |= CREAD | CLOCAL;

    // 非阻塞 read 超时
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1; // 0.1s

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        close(fd);
        return -1;
    }

    return fd;
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        fprintf(stderr, "Usage: %s <serial-device> <baud> [--crc]\n", argv[0]);
        return 1;
    }
    const char *dev = argv[1];
    int baud = atoi(argv[2]);
    for (int i = 3; i < argc; ++i) {
        if (strcmp(argv[i], "--crc") == 0) use_crc = true;
    }

    int fd = open_serial(dev, baud);
    if (fd < 0) return 2;
    printf("Opened %s @ %d (CRC=%s)\n", dev, baud, use_crc?"ON":"OFF");

    // 简单的缓冲区用于解析
    uint8_t buf[256];
    size_t buf_len = 0;

    while (1) {
        ssize_t r = read(fd, buf + buf_len, sizeof(buf) - buf_len);
        if (r < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                usleep(10000);
                continue;
            }
            perror("read");
            break;
        } else if (r == 0) {
            // 没有数据
            usleep(10000);
            continue;
        }
        buf_len += r;

        // 尝试解析帧：寻找 0xAA，然后检查是否有足够字节
        size_t i = 0;
        while (i < buf_len) {
            if (buf[i] != FRAME_START) { i++; continue; }
            size_t need = use_crc ? 12 : 10; // start + id + payload(8) + [crc] + end
            if (i + need > buf_len) break; // 等待更多字节
            if (buf[i + need - 1] != FRAME_END) {
                // 起始字节无效（尾部不匹配），丢弃该起始并继续
                i++;
                continue;
            }
            uint8_t id = buf[i+1];
            uint8_t payload[8];
            memcpy(payload, &buf[i+2], 8);
            if (use_crc) {
                uint8_t rcrc = buf[i+10];
                uint8_t calc = crc8((const uint8_t[]){id, payload[0], payload[1], payload[2], payload[3], payload[4], payload[5], payload[6], payload[7]}, 9, 0x07);
                if (calc != rcrc) {
                    printf("CRC mismatch: got %02x calc %02x\n", rcrc, calc);
                    i += 1; // skip this start
                    continue;
                }
            }
            // 处理帧
            handle_frame(fd, id, payload);
            // 移动缓冲区
            size_t remain = buf_len - (i + need);
            memmove(buf, buf + i + need, remain);
            buf_len = remain;
            i = 0; // 从头开始解析
        }
        // 防止缓冲区溢出
        if (buf_len == sizeof(buf)) buf_len = 0;
    }

    close(fd);
    return 0;
}
