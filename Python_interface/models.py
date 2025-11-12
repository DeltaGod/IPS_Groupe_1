# models.py
# coding: utf-8
DEBUG=False

from uart_manager import UARTmanager

from observer import Subject, Observer


class MainModel(Subject, Observer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.port = None
        self.baudrate = None
        self.current = None
        self.temperature = None
        self.duty_cycle = None
        self.power = None
        self.temperature_setpoint = None

        self.uart = UARTmanager()


    # ----- setters -----
    def set_port(self, port: str):
        self.uart.set_port(port)
        self.port = port
        self.notify()
    def set_baudrate(self, baudrate: float):
        self.uart.set_baudrate(baudrate)
        self.baudrate = baudrate
        self.notify()
    def set_temperature_setpoint(self, temperature_setpoint: float):
        # TODO: send temperature setpoint to UART
        self.temperature_setpoint = temperature_setpoint
        self.notify()
    def set_duty_cycle(self, port: float):
        self.port = port
        self.notify()
    def set_data(self, current, temperature, duty_cycle, power):
        self.current = current
        self.temperature = temperature
        self.duty_cycle = duty_cycle
        self.power = power
        self.notify()
    # ----- getters -----
    def get_port(self):
        return self.port
    def get_baudrate(self):
        return self.baudrate
    def get_current(self):
        return self.current
    def get_temperature(self):
        return self.temperature
    def get_duty_cycle(self):
        return self.duty_cycle
    def get_power(self):
        return self.power
    def get_temperature_setpoint(self):
        return self.temperature_setpoint

    def fetch_measures(self):
        data = self.uart.read_data()
        if data :
            if DEBUG:
                print(data)
            self.set_data(data["current"], 
                          data["temperature"],
                          data["duty_cycle"],
                          data["power"])
    
    def update(self, subject):
        self.fetch_measures()
    
    def reconnect_uart(self, port, baudrate):
        self.set_port(port)
        self.set_baudrate(baudrate)
        self.uart.connect()
    
    def cleanup(self):
        pass

