# main.py
# coding: utf-8
DEBUG=True
# Before running, make sure to:
# ip_set_can
# ifconfig
# in a separate terminal

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow

from views import MainView
from  controllers import MainController
from models import MainModel

class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.setWindowTitle("Thermistance Control")

        self.view = MainView() # QWidget
        self.model = MainModel()
        self.controller = MainController(self.view, self.model)

        self.setCentralWidget(self.view)

        self.model.attach(self.view)
        self.controller.bind_callbacks()
    
    
    def closeEvent(self, event):
        if DEBUG:
            print("Closing application...")
        self.cleanup()
        event.accept()  # allow the window to close

    def cleanup(self):
        if DEBUG:
            print("Cleaning up resources...")
        self.controller.cleanup()
        self.model.cleanup() # stop timer
        self.view.cleanup()

    


if __name__ == "__main__":

    from PyQt5.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())