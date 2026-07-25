#!/usr/bin/env python3
import serial
import time
import threading
import sys

def read_thread(ser):
    print("Read thread started")
    while True:
        try:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                if data:
                    hex_str = ' '.join([f'{b:02X}' for b in data])
                    try:
                        text_str = data.decode('utf-8', errors='ignore')
                    except:
                        text_str = "<binary>"
                    print(f"\n[RECV] {len(data)} bytes: {hex_str}")
                    print(f"[TEXT] {text_str}")
            else:
                time.sleep(0.01)
        except Exception as e:
            print(f"Read error: {e}")
            break

def main():
    port = "/dev/ttyACM0"
    baud = 115200
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
        print(f"Opened {port} @ {baud}")
    except Exception as e:
        print(f"Failed to open port: {e}")
        return

    t = threading.Thread(target=read_thread, args=(ser,), daemon=True)
    t.start()

    # Test frame: AA FF 01 00 00 00 00 00 00 00 55
    # ID=0xFF, Payload=01... (Control=1 -> Green LED)
    frame = bytes([0xAA, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55])
    
    print("Sending test frames (Ctrl=1, Green LED)... Press Ctrl+C to stop.")
    try:
        while True:
            ser.write(frame)
            print(".", end='', flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
