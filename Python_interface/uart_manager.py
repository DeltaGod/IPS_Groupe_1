# read_uart.py
'''
:Dxxx = change duty cycle
:S = start data stream
:S0 = stop data stream
:A = tout stopper (Dty cycle à 0 et stopper le data stream)
Courant, Température, Duty Cycle Actuel, Puissance estimée de la thermistance
\n = EOL -> to detect when 1 line of data ends
line format: Current;Temperature;DutyCycle;Power\n
'''
DEBUG = True

import serial
import time
import atexit
import sys


class UARTmanager():

    def __init__(self):
        self.duty_cycle = None
        self.streaming  = False
        self.port       = None # or "/dev/ttyUSB0" on Linux 
        # Adjust to your COM port or /dev/ttyUSBx
        self.baudrate   = None
        self.ser = None
        # Register automatic safety on program exit
        atexit.register(self.disconnect)
    
    def notify(self):
        pass
    
    # ----- setters -----
    def set_duty_cycle(self, duty_cycle: int):
        self.send_command(f":D{duty_cycle}\n")
        self.duty_cycle = duty_cycle
        self.notify()
    def set_port(self, port: str):
        self.disconnect()
        self.port = port
        self.notify()
    def set_baudrate(self, baudrate: int):
        self.disconnect()
        self.baudrate = baudrate
        self.notify()
    # ----- getters -----
    def get_duty_cycle(self):
        return self.duty_cycle
    def get_port(self):
        return self.port
    def get_baudrate(self):
        return self.baudrate
    # def get_ser(self):
    #     return self.ser
    def get_streaming_state(self):
        return self.streaming

    def start_streaming(self):
        if DEBUG:
            print("Starting data stream")
        self.send_command(":S\n")
        self.streaming = True
        self.notify()
        
    def stop_streaming(self):
        if DEBUG:
            print("Stopping data stream")
        self.send_command(":S0\n")
        self.streaming = False
        self.notify()
        
    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            if DEBUG:
                print(f"Connected to {self.port} at {self.baudrate} baud.")
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            sys.exit(1)
    
    def disconnect(self):
        self.stop_everything()
        time.sleep(0.2)  # allow STM32 time to process
        if self.ser and self.ser.is_open:
            if DEBUG:
                print("Closing UART connection...")
            self.ser.close()
            if DEBUG:
                print("Disconnected safely.")

    def send_command(self, command: str):
        command = str(command)
        if DEBUG:
            print(f"Sending: {command.strip()}")
        self.ser.write(command.encode())

    def stop_everything(self):
        if DEBUG:
            print("Stopping thermistance heating and data stream")
        self.send_command(":A\n")
        self.streaming = False
        self.notify()
    
    def readline(self):
        line=""
        if self.ser.in_waiting > 0:
            line = self.ser.readline().decode(errors='ignore').strip()
        if DEBUG and line!="":
            print(f">> {line}")
        return line
    
    def read_data(self):
        line = self.readline().split(";")
        data = None
        if len(line) > 3:
            data = {"current": line[0],
                    "temperature": line[1],
                    "duty cycle": line[2],
                    "power": line[3]}
        if DEBUG and data:
            print(data)
        return data
        


if __name__ == "__main__" :

    uart = UARTmanager(port="COM8")

    uart.connect()
    uart.start_streaming()
    uart.set_duty_cycle(50)

    while True:
        uart.get_data()