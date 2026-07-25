#!/usr/bin/env python3
"""
serial_echo.py

简单的串口回显守护，用于测试 `serial_frame_test.py`。
- 持续读取串口数据，按 Protocol C 解析帧。
- 当解析到合法帧时，把相同的帧原样写回（可选择 CRC）。

用法示例：
  python3 tools/serial_echo.py --port /dev/pts/4 --baud 115200 --crc

当 --crc 指定时，期望并返回带 CRC 的帧（帧格式：0xAA id payload(8) crc 0x55）。
否则使用无 CRC 的格式（帧格式：0xAA id payload(8) 0x55）。
"""

import argparse
import serial
import time


def crc8(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) & 0xFF) ^ poly
            else:
                crc = (crc << 1) & 0xFF
    return crc & 0xFF


def find_frame(buf: bytearray, use_crc: bool):
    """尝试在 buf 中找到一个完整帧，返回 (start_idx, end_idx, id, payload, crc_or_none)
    或者返回 None 表示未找到完整帧。"""
    try:
        start = buf.index(0xAA)
    except ValueError:
        return None
    if use_crc:
        total = 12  # 0xAA id(1) payload(8) crc(1) 0x55
    else:
        total = 10  # 0xAA id(1) payload(8) 0x55
    if start + total > len(buf):
        return None
    if buf[start + total - 1] != 0x55:
        # 如果尾部不对，丢弃该起始标记，继续
        del buf[start:start+1]
        return None
    # 解析
    rid = buf[start + 1]
    payload = bytes(buf[start + 2:start + 10])
    crc = None
    if use_crc:
        crc = buf[start + 10]
    end = start + total
    return (start, end, rid, payload, crc)


def main():
    parser = argparse.ArgumentParser(description="Serial echo daemon for Protocol C frames")
    parser.add_argument("--port", required=True, help="串口设备，例如 /dev/pts/4")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--crc", action="store_true", help="期望并返回带 CRC 的帧")
    args = parser.parse_args()

    with serial.Serial(args.port, args.baud, timeout=0.1) as ser:
        print(f"Opened {args.port} @ {args.baud}, CRC={'ON' if args.crc else 'OFF'}")
        buf = bytearray()
        try:
            while True:
                data = ser.read(256)
                if data:
                    buf.extend(data)
                    # 尝试查找并处理所有完整帧
                    while True:
                        found = find_frame(buf, args.crc)
                        if not found:
                            break
                        start, end, rid, payload, crc = found
                        # 移除该帧
                        frame = bytes(buf[start:end])
                        del buf[start:end]
                        # 校验 CRC（如启用）
                        if args.crc:
                            calc = crc8(bytes([rid]) + payload)
                            if calc != crc:
                                print(f"CRC mismatch for frame id=0x{rid:02x}: got {crc:02x}, calc {calc:02x}")
                                continue
                        print(f"Received frame id=0x{rid:02x} payload={payload.hex(' ')}")
                        # 原样回写（保持 CRC/尾部格式）
                        ser.write(frame)
                        ser.flush()
                        print(f"Echoed back {len(frame)} bytes")
                else:
                    time.sleep(0.01)
        except KeyboardInterrupt:
            print("Exiting")


if __name__ == '__main__':
    main()
