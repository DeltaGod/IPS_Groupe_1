# models.py
# coding: utf-8
DEBUG = True

from uart_manager import UARTmanager
from observer import Subject, Observer
from local_timer import Timer

class MainModel(Subject, Observer):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._port = None
        self._baudrate = None
        self._is_connected = False
        self._is_running = False

        self._refreshing_frequency = 20 # Hz
        self._timer = Timer(1/self._refreshing_frequency)
        self._timer.attach(self)

        self._temperature = None
        self._current = None
        self._duty_cycle = None
        self._power = None
        self._temperature_setpoint = None

        self._uart = UARTmanager()

    # ----- setters -----
    def set_port(self, port: str):
        self._uart.set_port(port)
        self._port = port
        self.notify()
    def set_baudrate(self, baudrate: float):
        self._uart.set_baudrate(baudrate)
        self._baudrate = baudrate
        self.notify()
    def set_is_connected(self, is_connected):
        '''connect/disconnect features implemented here'''
        if is_connected:
            if (self._port is None) or (self._baudrate is None):
                if DEBUG:
                    print(f"{type(self).__name__}.set_is_connected() - Missing port or baudrate!")
                self._is_connected = False
                raise ConnectionAbortedError
            elif self._is_connected: 
                if DEBUG:
                    print(f"{type(self).__name__}.set_is_connected() - Already connected")
            else :
                try :
                    self._uart.connect()
                    self._is_connected = True
                except:
                    raise ConnectionError
        else:
            self._uart.disconnect()
            self._is_connected = False
        self.notify()
    def set_is_running(self, is_running):
        '''start/stop features implemented here'''
        if is_running:
            # send setpoint and start data_stream
            if not self._is_connected:
                if DEBUG:
                    print(f"{type(self).__name__}.set_is_running() - Not Connected!")
                self._is_running = False
                self._timer.stop()
                pass
            elif self._is_running:
                if DEBUG:
                    print(f"{type(self).__name__}.set_is_running() - Already running")
                self._timer.start()
            else:
                self._uart.start_streaming()
                self._is_running = True
                self._timer.start()
        else:
            self._uart.stop_everything()
            self._is_running = False
            self._timer.stop()
        self.notify()
    def set_refreshing_frequency(self, frequency):
        self._timer.period = 1/frequency
        self._refreshing_frequency = frequency
        self.notify()
    def set_temperature_setpoint(self, temperature_setpoint: float):
        '''
        temperature: float between 0.0-999.9 (°C), precision is 1 decimal only  
        Tries to set the temperature setpoint of the card. If something fails this is a no-op.
        '''
        self._uart.set_temperature_setpoint(temperature_setpoint)
        # self._temperature_setpoint = temperature_setpoint
        self._temperature_setpoint = self._uart.get_temperature_setpoint()
        self.notify()
    def set_data(self, current, temperature, duty_cycle, power):
        self._current = current
        self._temperature = temperature
        self._duty_cycle = duty_cycle
        self._power = power
        self.notify()
    def set_duty_cycle(self, duty_cycle: float):
        self._uart.set_duty_cycle(duty_cycle)
        self._duty_cycle = duty_cycle
        self.notify()
    # ----- getters -----
    def get_port(self):
        return self._port
    def get_baudrate(self):
        return self._baudrate
    def get_is_running(self):
        return self._is_running
    def get_is_connected(self):
        return self._is_connected
    def get_temperature(self):
        return self._temperature
    def get_current(self):
        return self._current
    def get_duty_cycle(self):
        return self._duty_cycle
    def get_power(self):
        return self._power
    def get_temperature_setpoint(self):
        return self._temperature_setpoint

    # FUNCTIONNALITY METHODS
    def fetch_measures(self):
        data = self._uart.read_data()
        if DEBUG:
            print(f"{type(self).__name__}.fetch_measures()")
        if data :
            current = data["current"]
            temperature = data["temperature"]
            duty_cycle = data["duty_cycle"]
            power = data["power"]
            if DEBUG:
                print(f"{type(self).__name__}.fetch_measures() - {data}")
                print(f"{type(self).__name__}.fetch_measures() - {current}, {temperature}, {duty_cycle}, {power}")
            self.set_data(current, temperature, duty_cycle, power)
    
    def update(self, subject):
        if DEBUG:
            print(f"{type(self).__name__}.update()")
        self.fetch_measures()
    
    def cleanup(self):
        # TODO
        pass

if __name__ == "__main__":
    SCENARIO = 1

    print(">> Testing model.py <<")

    if SCENARIO==1:
        testmodel = MainModel()

    else:
        print("Nothing to test.")