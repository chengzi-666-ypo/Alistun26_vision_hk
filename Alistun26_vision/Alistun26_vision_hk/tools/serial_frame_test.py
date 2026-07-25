#!/usr/bin/env python3
"""
serial_frame_test.py

简单串口帧测试脚本（Protocol C）
帧格式: 0xAA | id(1 byte) | payload(8 bytes) | [crc?] | 0x55

特点:
- 支持可选 CRC-8（默认关闭），CRC 计算在 id+payload 上并在 payload 后附加一个 CRC 字节
- 支持 --dump 打印逐字节收发十六进制

用途:
- 向 C-board 发送测试命令帧
- 可选择性读取并打印从串口返回的数据并校验

依赖:
- pyserial (pip install pyserial)

示例用法:
  python3 tools/serial_frame_test.py --port /dev/ttyACM0 --baud 115200 --id 0x01 --payload "00 01 02 03 04 05 06 07" --read --dump

如果需要在没有硬件的情况下测试，可以用 socat 在两个虚拟串口间桥接：
  socat -d -d pty,raw,echo=0 pty,raw,echo=0

脚本会打印已发送的字节以及收到的响应（如果 --read 指定）。
"""

import argparse
import serial
import time


def parse_payload(s: str) -> bytes:
    """解析用户输入的 payload，比如 "00 01 02 ..." 或连续的 hex 字符串"""
    s = s.replace("0x", "").replace(",", " ")
    parts = s.split()
    if len(parts) == 1 and len(parts[0]) == 16:
        # 连续 hex 16 字符 -> 分成 8 字节
        parts = [parts[0][i:i+2] for i in range(0, 16, 2)]
    if len(parts) != 8:
        raise ValueError("payload 必须为 8 个字节（16 个 hex 字符或 8 个用空格分隔的字节）")
    return bytes(int(p, 16) for p in parts)


def build_frame(fid: int, payload: bytes) -> bytes:
    assert 0 <= fid <= 0xFF
    assert len(payload) == 8
    # 格式: 0xAA | ID | Payload(8) | 0x55
    frame = bytes([0xAA, fid]) + payload + bytes([0x55])
    return frame


def main():
    parser = argparse.ArgumentParser(description="Send a Protocol C frame over serial and optionally read response.")
    parser.add_argument("--port", required=True, help="串口设备，例如 /dev/ttyUSB0 或 /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200, help="波特率（默认 115200）")
    parser.add_argument("--id", required=True, help="帧 ID，例如 0x01 或 1")
    parser.add_argument("--payload", default="00 00 00 00 00 00 00 00", help="8 字节 payload（hex），例如 '00 01 02 03 04 05 06 07' 或 '0001020304050607'")
    parser.add_argument("--read", action="store_true", help="发送后读取并打印返回（短超时）")
    parser.add_argument("--timeout", type=float, default=0.5, help="读取超时（秒）")
    parser.add_argument("--dump", action="store_true", help="逐字节打印收发数据（十六进制）")

    args = parser.parse_args()

    # 解析 id
    if isinstance(args.id, str) and args.id.startswith("0x"):
        fid = int(args.id, 16)
    else:
        fid = int(args.id)

    payload = parse_payload(args.payload)
    frame = build_frame(fid, payload)

    print(f"Opening serial {args.port} @ {args.baud}...")
    with serial.Serial(args.port, args.baud, timeout=args.timeout) as ser:
        if args.dump:
            print(f"[DUMP] TX: {frame.hex(' ')}")
        print(f"Opened. Sending frame: {frame.hex(' ')}")
        ser.write(frame)
        ser.flush()

        if args.read:
            # 等待少许时间让对端回包
            time.sleep(0.05)
            resp = ser.read(1024)
            if resp:
                if args.dump:
                    print(f"[DUMP] RX RAW: {resp.hex(' ')}")

                # 查找 0xAA ... 0x55
                try:
                    # 简单的帧查找逻辑
                    start = resp.index(0xAA)
                    # 寻找匹配的结束符，最小帧长 11 (1+1+8+1)
                    # 从 start + 10 开始找 0x55
                    if start + 10 < len(resp) and resp[start + 10] == 0x55:
                        rid = resp[start + 1]
                        rpayload = resp[start + 2 : start + 10]
                        print(f"Received frame id=0x{rid:02x} payload={rpayload.hex(' ')}")
                    else:
                        # 尝试寻找下一个 0x55
                        try:
                            end = resp.index(0x55, start + 1)
                            print(f"Received partial/malformed frame: {resp[start:end+1].hex(' ')}")
                        except ValueError:
                            print(f"Received raw (no end byte): {resp.hex(' ')}")
                except ValueError:
                    print(f"Received raw (no start byte): {resp.hex(' ')}")
            else:
                print("No response received (timeout)")


if __name__ == '__main__':
    main()
