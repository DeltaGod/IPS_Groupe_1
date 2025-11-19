# uart_manager.py
# coding: utf-8
DEBUG = True

import serial
import time
import atexit
import sys

from observer import Subject

'''
## UART Communication commands:
    :Dxxx = change duty cycle
    :S = start data stream
    :S0 = stop data stream
    :A = stop everything (duty_cycle=0 & stop data stream)
## UART data formatting
    \n = EOL -> to detect when 1 line of data ends
    line format: Current;Temperature;DutyCycle;Power\n
'''
class UARTmanager(Subject):

    def __init__(self):
        super().__init__()
        self._duty_cycle    = None
        self._temperature_setpoint   = None
        self._streaming     = False
        self._port          = None # Adjust to your COM port or /dev/ttyUSBx
        self._baudrate      = None # standard is 115200
        self._ser           = None # serial connexion with UART
        atexit.register(self.disconnect) # Register automatic safety on program exit
    
    # ----- setters -----
    def set_duty_cycle(self, duty_cycle: int):
        '''duty_cycle: int between 0-100 (%)'''
        self.send_command(f":D{duty_cycle}\n")
        self._duty_cycle = duty_cycle
        self.notify()
    def set_temperature_setpoint(self, temperature: float):
        '''temperature: float between 0.0-999.9, precision is 1 decimal only'''
        temperature=int(temperature*10)
        self.send_command(f":X{temperature}\n")
        self._temperature_setpoint = temperature/10
        self.notify()
    def set_port(self, port: str):
        '''port: "COM0" or "COMX" on windows, "/dev/ttyUSB0" or "/dev/ttyUSBx" on linux'''
        self.disconnect()
        self._port = port
        self.notify()
    def set_baudrate(self, baudrate: int):
        '''baudrate: standard is 115200 (Hz)'''
        self.disconnect()
        self._baudrate = baudrate
        self.notify()
    # ----- getters -----
    def get_duty_cycle(self):
        return self._duty_cycle
    def get_temperature_setpoint(self):
        return self._temperature_setpoint
    def get_port(self):
        return self._port
    def get_baudrate(self):
        return self._baudrate
    # def get_ser(self):
    #     return self._ser
    def get_streaming_state(self):
        return self._streaming

    def start_streaming(self):
        if DEBUG:
            print(f"{type(self).__name__}.start_streaming()")
        self.send_command(":S\n")
        self._streaming = True
        self.notify()
        
    def stop_streaming(self):
        if DEBUG:
            print(f"{type(self).__name__}.stop_streaming()")
        self.send_command(":S0\n")
        self._streaming = False
        self.notify()
        
    def connect(self):
        try:
            self._ser = serial.Serial(self._port, self._baudrate, timeout=1)
            if DEBUG:
                print(f"{type(self).__name__}.connect() - Connected to {self._port} at {self._baudrate} baud.")
            time.sleep(2)
        except serial.SerialException as e:
            print(f"Serial error: {e}")
            sys.exit(1)
    
    def disconnect(self):
        self.stop_everything()
        time.sleep(0.2)  # allow STM32 time to process
        if self._ser and self._ser.is_open:
            if DEBUG:
                print(f"{type(self).__name__}.disconnect() - Closing UART connection...")
            self._ser.close()
            if DEBUG:
                print(f"{type(self).__name__}.disconnect() - Disconnected safely.")

    def send_command(self, command: str):
        command = str(command)
        if self._ser is not None:
            if DEBUG:
                print(f"{type(self).__name__}.send_command() - Sending: {command.strip()}")
            self._ser.write(command.encode())

    def stop_everything(self):
        if DEBUG:
            print(f"{type(self).__name__}.stop_everything() - Stopping thermistance heating and data stream")
        self.send_command(":A\n")
        self._streaming = False
        self.notify()
    
    def readline(self):
        line=""
        if self._ser.in_waiting > 0:
            line = self._ser.readline().decode(errors='ignore').strip()
        if DEBUG and line!="":
            print(f"{type(self).__name__}.readline() >> {line}")
        return line
    
    def read_data(self):
        line = self.readline().split(";")
        data = None
        if len(line) > 3:
            data = {"current": line[0],
                    "temperature": line[1],
                    "duty_cycle": line[2],
                    "power": line[3]}
        if DEBUG and data:
            print(f"{type(self).__name__}.read_data() - {data}")
        return data
        


if __name__ == "__main__" :

    uart = UARTmanager()

    uart._baudrate = 115200
    uart._port = "COM8"

    uart.connect()
    uart.start_streaming()
    uart.set_duty_cycle(50)

    while True:
        uart.read_data()