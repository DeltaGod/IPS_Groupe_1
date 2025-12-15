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
from PyQt5.QtCore import QSize, Qt
import pyqtgraph as pg # for graphs

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
        self.setMinimumSize(790, 350)
        # Fonts parameters
        self.fontfamily = "Arial"
        self.titlefont = QFont(self.fontfamily, 12, QFont.Weight.Bold)
        self.labelfont = QFont(self.fontfamily, 10)
        self.notefont = QFont(self.fontfamily, 18)
        labels_width = 140
        labels_height = 50
        data_width = 80
        title_width = 250
        title_height = 40

        ## ---------------
        ## DATA WIDGETS
        ## ---------------
        # Title
        self.data_section_title = QLabel("MEASURED DATA")
        self.data_section_title.setMinimumSize(title_width, title_height)
        self.data_section_title.setMaximumSize(title_width+50, title_height+20)
        # Temperature
        self.temperature_label = QLabel("Temperature")
        self.temperature_label.setMinimumSize(labels_width, labels_height)
        self.temperature_label.setMaximumSize(labels_width+50, labels_height+20)
        self.temperature_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.temperature_label.setContentsMargins(5, 10, 5, 5)
        self.temperature_data = QLabel("-- °C")
        self.temperature_data.setMinimumSize(data_width, labels_height)
        self.temperature_data.setMaximumSize(data_width+50, labels_height+20)
        self.temperature = QHBoxLayout()
        self.temperature.addWidget(self.temperature_label)
        self.temperature.addWidget(self.temperature_data)
        # Current
        self.current_label = QLabel("Current")
        self.current_label.setMinimumSize(labels_width, labels_height)
        self.current_label.setMaximumSize(labels_width+50, labels_height+20)
        self.current_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.current_label.setContentsMargins(5, 10, 5, 5)
        self.current_data = QLabel("-- A")
        self.current_data.setMinimumSize(data_width, labels_height)
        self.current_data.setMaximumSize(data_width+50, labels_height+20)
        self.current = QHBoxLayout()
        self.current.addWidget(self.current_label)
        self.current.addWidget(self.current_data)
        # Duty Cycle
        self.duty_cycle_label = QLabel("Duty Cycle")
        self.duty_cycle_label.setMinimumSize(labels_width, labels_height)
        self.duty_cycle_label.setMaximumSize(labels_width+50, labels_height+20)
        self.duty_cycle_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.duty_cycle_label.setContentsMargins(5, 10, 5, 5)
        self.duty_cycle_data = QLabel("-- %")
        self.duty_cycle_data.setMinimumSize(data_width, labels_height)
        self.duty_cycle_data.setMaximumSize(data_width+50, labels_height+20)
        self.duty_cycle = QHBoxLayout()
        self.duty_cycle.addWidget(self.duty_cycle_label)
        self.duty_cycle.addWidget(self.duty_cycle_data)
        # Power
        self.power_label = QLabel("Power")
        self.power_label.setMinimumSize(labels_width, labels_height)
        self.power_label.setMaximumSize(labels_width+50, labels_height+20)
        self.power_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.power_label.setContentsMargins(5, 10, 5, 5)
        self.power_data = QLabel("--W")
        self.power_data.setMinimumSize(data_width, labels_height)
        self.power_data.setMaximumSize(data_width+50, labels_height+20)
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
        # Style Sheet
        self.data_widget.setObjectName("data_widget")
        self.data_widget.setStyleSheet("#data_widget { border: 1px solid black; }")
        self.data_section_title.setStyleSheet("border: 1px solid black;")
        self.temperature_data.setStyleSheet("border: 1px solid lightgrey;")
        self.current_data.setStyleSheet("border: 1px solid lightgrey;")
        self.duty_cycle_data.setStyleSheet("border: 1px solid lightgrey;")
        self.power_data.setStyleSheet("border: 1px solid lightgrey;")

        ## ---------------
        ## SETPOINT CONTROL WIDGET
        ## ---------------
        # Title
        self.ctrls_section_title = QLabel("THERMISTANCE CONTROL")
        self.ctrls_section_title.setMinimumSize(title_width, title_height)
        # self.ctrls_section_title.setMaximumSize(150+50, 40)
        self.ctrls_section_title.setMaximumHeight(title_height+20)
        self.ctrls_section_title.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        self.ctrls_section_title.setContentsMargins(5, 5, 5, 5)
        # Temperature setpoint
        self.temperature_setpoint_label = QLabel("Setpoint")
        self.temperature_setpoint_label.setMinimumSize(labels_width, labels_height)
        self.temperature_setpoint_label.setMaximumSize(labels_width+50, labels_height+20)
        self.temperature_setpoint_label.setAlignment(Qt.AlignmentFlag.AlignRight)
        self.temperature_setpoint_label.setContentsMargins(5, 10, 5, 5)
        self.temperature_setpoint_display = QLabel("-- °C")
        self.temperature_setpoint_display.setStyleSheet("border: 1px solid lightgrey;")
        self.temperature_setpoint_display.setMinimumSize(data_width, labels_height)
        self.temperature_setpoint_display.setMaximumSize(data_width+50, labels_height+20)
        self.temperature_setpoint_inputbox = QLineEdit("")
        self.temperature_setpoint_inputbox.setMinimumSize(data_width, labels_height)
        self.temperature_setpoint_inputbox.setMaximumSize(data_width+50, labels_height+20)
        self.temperature_setpoint_inputbutton = QPushButton("set")
        self.temperature_setpoint_inputbutton.setMinimumSize(50, 0)
        self.temperature_setpoint_inputbutton.setMaximumSize(50+50, 40)
        self.temperature_setpoint = QHBoxLayout()
        self.temperature_setpoint.addWidget(self.temperature_setpoint_label)
        self.temperature_setpoint.addWidget(self.temperature_setpoint_display)
        self.temperature_setpoint.addWidget(self.temperature_setpoint_inputbox)
        self.temperature_setpoint.addWidget(self.temperature_setpoint_inputbutton)
        # buttons
        self.off_button = QPushButton("OFF")
        self.off_button.setCheckable(True)
        self.off_button.setMinimumSize(50, 0)
        self.off_button.setMaximumSize(50+50, 40)
        self.on_button = QPushButton("ON")
        self.on_button.setCheckable(True)
        self.on_button.setMinimumSize(50, 0)
        self.on_button.setMaximumSize(50+50, 40)
        self.buttons = QHBoxLayout()
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
        # Style Sheet
        self.ctrls.setObjectName("ctrls")
        self.ctrls.setStyleSheet("#ctrls { border: 1px solid black; }")
        self.ctrls_section_title.setStyleSheet("border: 1px solid black;")
        
        ## ---------------
        ## GRAPHS WIDGET
        ## ---------------
        self.temperature_graph = pg.PlotWidget()
        self.current_graph = pg.PlotWidget()
        self.duty_cycle_graph = pg.PlotWidget()
        self.power_graph = pg.PlotWidget()

        ## ---------------
        ## OVERALL LAYOUT
        ## ---------------
        self.mainlayout = QGridLayout()
        self.mainlayout.addWidget(self.data_widget, 0, 0, 1, 1)
        self.mainlayout.addWidget(self.ctrls, 0, 1, 1, 1)
        self.setLayout(self.mainlayout)
    
    def update(self, subject):
        # self.set_temperature_data(subject.get_temperature())
        # self.set_current_data(subject.get_current())
        # self.set_duty_cycle_data(subject.get_duty_cycle())
        # self.set_power_data(subject.get_power())
        self.set_data(subject.get_temperature(),
                      subject.get_current(),
                      subject.get_duty_cycle(),
                      subject.get_power())
        self.update_graphs(subject.get_temperature_list(),
                           subject.get_current_list(),
                           subject.get_duty_cycle_list(),
                           subject.get_power_list())
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
    def set_current_data(self, current: int|None):
        if current is not None:
            text = str(f"{current} °A")
        else: 
            text = "--"
        self.current_data.setText(text)
    def set_duty_cycle_data(self, duty_cycle: int|None):
        if duty_cycle is not None:
            text = str(f"{duty_cycle} %")
        else: 
            text = "--"
        self.duty_cycle_data.setText(text)
    def set_power_data(self, power: int|None):
        if power is not None:
            text = str(f"{power} W")
        else: 
            text = "--"
        self.power_data.setText(text)
    def set_data(self, temperature, current, duty_cycle, power):
        self.set_temperature_data(temperature=temperature)
        self.set_current_data(current=current)
        self.set_duty_cycle_data(duty_cycle=duty_cycle)
        self.set_power_data(power=power)

    def get_typed_temperature_setpoint(self):
        text = self.temperature_setpoint_inputbox.text()
        if text == "":
            return None
        try:
            setpoint = float(text)
            if DEBUG:
                print(f"{__name__}.get_typed_temperature_setpoint() -> {setpoint}")
            return setpoint
        except ValueError:
            return None
    
    def clear_temperature_setpoint_inputbox(self):
        self.temperature_setpoint_inputbox.setText("")
    
    def update_graphs(self, temperature_list, current_list, duty_cycle_list, power_list):
        time = range(0,100)
        self.temperature_graph.plot(time, temperature_list)
        self.current_graph.plot(time, current_list)
        self.duty_cycle_graph.plot(time, duty_cycle_list)
        self.power_graph.plot(time, power_list)
        

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

