import serial.tools.list_ports

def list_ports():
    ports = serial.tools.list_ports.comports()
    print(f"{'Device':<20} {'HWID':<40} {'Description'}")
    print("-" * 80)
    for port in ports:
        print(f"{port.device:<20} {port.hwid:<40} {port.description}")

if __name__ == "__main__":
    list_ports()
