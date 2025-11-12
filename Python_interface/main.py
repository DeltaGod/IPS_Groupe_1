# main.py
# coding: utf-8
DEBUG=True

import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMenuBar


from views import MainView
from controllers import MainController
from models import MainModel

class MainWindow(QMainWindow):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._createMenuBar()

        self.setWindowTitle("Thermistance Control")

        self.view = MainView() # QWidget
        self.model = MainModel()
        self.controller = MainController(self.view, self.model)

        self.setCentralWidget(self.view)

        self.model.attach(self.view)
        self.controller.bind_callbacks()
    
    
    def closeEvent(self, event):
        if DEBUG:
            print(f"{type(self).__name__}.closeEvent() - Closing application...")
        self.cleanup()
        event.accept()  # allow the window to close

    def cleanup(self):
        if DEBUG:
            print(f"{type(self).__name__}.cleanup() - Cleaning up resources...")
        self.controller.cleanup()
        self.model.cleanup() # stop timer
        self.view.cleanup()


    def _createMenuBar(self):
        menuBar = QMenuBar(self)
        menuBar.addMenu("menu")
        self.setMenuBar(menuBar)

    


if __name__ == "__main__":

    from PyQt5.QtWidgets import QApplication
    import sys

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    # force connexion to UART
    # TODO: once the port selection menu is done, remove these lines
    window.model.set_baudrate(115200)
    window.model.set_port("COM8")
    window.model.set_is_connected(True)

    sys.exit(app.exec())