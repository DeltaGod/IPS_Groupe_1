# controllers.py
# coding: utf-8
DEBUG=False

# from can_bus_manager import CanBusManager

class MainController:
    def __init__(self, view, model):
        self.view = view
        self.model = model
        self.lux_dist=True
    
    def bind_callbacks(self):
        self.view.mtr_off_button.clicked.connect(self.mtr_off_btn_clicked)
        self.view.mtr_auto_button.clicked.connect(self.mtr_auto_btn_clicked)
        self.view.mtr_on_button.clicked.connect(self.mtr_on_btn_clicked)
        self.view.lux_dist_button.clicked.connect(self.switch_light_dist_btn_clicked)

    def mtr_off_btn_clicked(self):
        self.model.set_motor_mode(0)

    def mtr_on_btn_clicked(self):
        self.model.set_motor_mode(1)

    def mtr_auto_btn_clicked(self):
        self.model.set_motor_mode(2)

    def switch_light_dist_btn_clicked(self):
        self.model.switch_LIGHT_MODE()

    def fetch_data(self):
        raise NotImplementedError

    def cleanup(self):
        pass

    

if __name__ == "__main__":
    SCENARIO = 1

    print(">> Testing controllers.py <<")

    if SCENARIO==1:
        from views import MainWindow
        view = MainWindow()
        controller = MainController(view)
    else:
        print("Nothing to test.")