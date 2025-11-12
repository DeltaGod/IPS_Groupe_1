# views.py
# coding: utf-8
DEBUG=False


import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QAction, QFileSystemModel, QMainWindow, 
    QPushButton, QWidget, QHBoxLayout, QVBoxLayout, QLabel, QLineEdit, 
    QGridLayout, QComboBox, QTreeView, QFileDialog, QMessageBox, 
    QInputDialog, QDialogButtonBox, QDialog, QGroupBox, QDoubleSpinBox,
    QDial, QSizePolicy, QSlider, QFrame
)
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QSize

from observer import Observer

## ============
## MAIN VIEW
## ============
class MainView(Observer, QWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        ## -----------------
        ## VISUAL PARAMETERS
        ## -----------------
        self.setMinimumSize(900, 300)
        # Fonts parameters
        self.fontfamily = "Arial"
        self.titlefont = QFont(self.fontfamily, 12, QFont.Weight.Bold)
        self.labelfont = QFont(self.fontfamily, 10)
        self.notefont = QFont(self.fontfamily, 18)

        ## ---------------
        ## DATA WIDGETS
        ## ---------------
        # Title
        self.data_section_title = QLabel("MEASURED DATA")
        # Temperature
        self.temperature_label = QLabel("Temperature")
        self.temperature_data = QLabel("-- °C")
        self.temperature_data.setMinimumWidth(100)
        self.temperature = QHBoxLayout()
        self.temperature.addWidget(self.temperature_label)
        self.temperature.addWidget(self.temperature_data)
        # Current
        self.current_label = QLabel("Current")
        self.current_data = QLabel("-- A")
        self.current_data.setMinimumWidth(100)
        self.current = QHBoxLayout()
        self.current.addWidget(self.current_label)
        self.current.addWidget(self.current_data)
        # Duty Cycle
        self.duty_cycle_label = QLabel("Duty Cycle")
        self.duty_cycle_data = QLabel("-- %")
        self.duty_cycle_data.setMinimumWidth(100)
        self.duty_cycle = QHBoxLayout()
        self.duty_cycle.addWidget(self.duty_cycle_label)
        self.duty_cycle.addWidget(self.duty_cycle_data)
        # Power
        self.power_label = QLabel("Power")
        self.power_data = QLabel("--W")
        self.power_data.setMinimumWidth(100)
        self.power = QHBoxLayout()
        self.power.addWidget(self.power_label)
        self.power.addWidget(self.power_data)
        # Layout
        self.data_layout = QVBoxLayout()
        self.data_layout.addWidget(self.data_section_title)
        self.data_layout.addLayout(self.temperature)
        self.data_layout.addLayout(self.current)
        self.data_layout.addLayout(self.duty_cycle)
        self.data_layout.addLayout(self.power)
        self.data_widget = QWidget()
        self.data_widget.setLayout(self.data_layout)
        self.data_section = QGroupBox("DATA BOX")

        ## ---------------
        ## SETPOINT CONTROL WIDGET
        ## ---------------
        # Title
        self.ctrls_section_title = QLabel("THERMISTANCE CONTROL")
        # Temperature setpoint
        self.temperature_setpoint_label = QLabel("Setpoint")
        self.temperature_setpoint_display = QLabel("-- °C")
        self.temperature_setpoint_display.setFixedWidth(100)
        self.temperature_setpoint_inputbox = QLineEdit("")
        self.temperature_setpoint_inputbutton = QPushButton("set")
        self.temperature_setpoint = QHBoxLayout()
        self.temperature_setpoint.addWidget(self.temperature_setpoint_label)
        self.temperature_setpoint.addWidget(self.temperature_setpoint_display)
        self.temperature_setpoint.addWidget(self.temperature_setpoint_inputbox)
        self.temperature_setpoint.addWidget(self.temperature_setpoint_inputbutton)
        # buttons
        self.off_button = QPushButton("OFF")
        self.off_button.setCheckable(True)
        self.on_button = QPushButton("ON")
        self.on_button.setCheckable(True)
        self.buttons = QVBoxLayout()
        self.buttons.addWidget(self.off_button)
        self.buttons.addWidget(self.on_button)
        self.init_buttons_state()
        # layout
        self.ctrls_layout = QVBoxLayout()
        self.ctrls_layout.addWidget(self.ctrls_section_title)
        self.ctrls_layout.addLayout(self.temperature_setpoint)
        self.ctrls_layout.addLayout(self.buttons)
        self.ctrls = QWidget()
        self.ctrls.setLayout(self.ctrls_layout)

        ## ---------------
        ## OVERALL LAYOUT
        ## ---------------
        self.mainlayout = QGridLayout()
        self.mainlayout.addWidget(self.data_widget, 0, 0, 1, 1)
        self.mainlayout.addWidget(self.ctrls, 0, 1, 1, 1)
        self.setLayout(self.mainlayout)

        self.setFixedWidth(1500)
    
    def update(self, subject):
        self.set_temperature_data(subject.get_temperature())
        self.set_current_data(subject.get_current())
        self.set_duty_cycle_data(subject.get_duty_cycle())
        self.set_power_data(subject.get_power())
        self.set_temperature_setpoint(subject.get_temperature_setpoint())
        self.set_buttons_state(subject.get_is_running())
    
    def init_buttons_state(self):
        self.off_button.setChecked(True)
        self.off_button.setEnabled(False)
        self.on_button.setChecked(False)
        self.on_button.setEnabled(True)
    def set_buttons_state(self, is_running: bool):
        if is_running:
            self.off_button.setChecked(False)
            self.off_button.setEnabled(True)
            self.on_button.setChecked(True)
            self.on_button.setEnabled(False)
        else:
            self.off_button.setChecked(True)
            self.off_button.setEnabled(False)
            self.on_button.setChecked(False)
            self.on_button.setEnabled(True)
    
    def cleanup(self):
        pass

    def set_temperature_setpoint(self, temperature: int | None):
        if temperature is not None:
            text = str(f"{temperature} °C")
        else: 
            text = "--"
        self.temperature_setpoint_display.setText(text)
    
    def set_temperature_data(self, temperature: int|None):
        if temperature is not None:
            text = str(f"{temperature} °C")
        else: 
            text = "--"
        self.temperature_data.setText(text)
        pass

    def set_current_data(self, current: int|None):
        if current is not None:
            text = str(f"{current} °A")
        else: 
            text = "--"
        self.current_data.setText(text)
        pass

    def set_duty_cycle_data(self, duty_cycle: int|None):
        if duty_cycle is not None:
            text = str(f"{duty_cycle} %")
        else: 
            text = "--"
        self.duty_cycle_data.setText(text)
        pass

    def set_power_data(self, power: int|None):
        if power is not None:
            text = str(f"{power} W")
        else: 
            text = "--"
        self.power_data.setText(text)
        pass

## ======================
## TESTING SOME STUFF
## ======================

if __name__ == "__main__":
    SCENARIO = 1

    print(">> Testing views.py <<")
    if SCENARIO==1:
        from PyQt5.QtWidgets import QApplication
        import sys

        app = QApplication(sys.argv)

        window = MainView()
        window.show()

        sys.exit(app.exec())

    elif SCENARIO==2:
        pass

    else:
        print("Nothing to test.")

