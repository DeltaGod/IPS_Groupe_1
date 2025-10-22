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
        self.setMinimumSize(300, 300)
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
        # Current
        self.current_label = QLabel("Current")
        self.current_data = QLineEdit("")
        self.current_data.setReadOnly(True)
        self.current = QHBoxLayout()
        self.current.addWidget(self.current_label)
        self.current.addWidget(self.current_data)
        # Temperature
        self.temperature_label = QLabel("Temperature")
        self.temperature_data = QLineEdit("")
        self.temperature_data.setReadOnly(True)
        self.temperature = QHBoxLayout()
        self.temperature.addWidget(self.temperature_label)
        self.temperature.addWidget(self.temperature_data)
        # Duty Cycle
        self.duty_cycle_label = QLabel("Duty Cycle")
        self.duty_cycle_data = QLineEdit("")
        self.duty_cycle_data.setReadOnly(True)
        self.duty_cycle = QHBoxLayout()
        self.duty_cycle.addWidget(self.duty_cycle_label)
        self.duty_cycle.addWidget(self.duty_cycle_data)
        # Power
        self.power_label = QLabel("Power")
        self.power_data = QLineEdit("")
        self.power_data.setReadOnly(True)
        self.power = QHBoxLayout()
        self.power.addWidget(self.power_label)
        self.power.addWidget(self.power_data)
        # Layout
        self.data_layout = QVBoxLayout()
        self.data_layout.addWidget(self.data_section_title)
        self.data_layout.addLayout(self.current)
        self.data_layout.addLayout(self.temperature)
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
        self.temperature_setpoint_label = QLabel("Temperature setpoint")
        self.temperature_setpoint_display = QLineEdit("")
        self.temperature_setpoint_display.setReadOnly(True)
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
    
    def update(self, subject):
        pass
    
    def init_buttons_state(self):
        self.off_button.setChecked(True)
        self.off_button.setEnabled(False)
    
    def cleanup(self):
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

