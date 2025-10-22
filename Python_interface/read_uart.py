# read_uart.py

import serial
import time

# Adjust to your COM port or /dev/ttyUSBx
PORT = "COM3"  # or "/dev/ttyUSB0" on Linux
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"Connected to {PORT} at {BAUDRATE} baud.")
    time.sleep(2)  # Wait a bit for STM32 to boot

    while True:
        if ser.in_waiting > 0:
            line = ser.readline().decode(errors='ignore').strip()
            if line:
                print(f"Received: {line}")

except serial.SerialException as e:
    print(f"Serial error: {e}")
except KeyboardInterrupt:
    print("Exiting...")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
