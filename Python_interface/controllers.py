# controllers.py
# coding: utf-8
DEBUG=False

class MainController:
    def __init__(self, view, model):
        self.view = view
        self.model = model
        self.model.attach(self.view)
    
    def bind_callbacks(self):
        if DEBUG :
            print(f"{type(self).__name__}")
        self.view.on_button.clicked.connect(self.on_btn_clicked)
        self.view.off_button.clicked.connect(self.off_btn_clicked)

    def temperature_setpoint_btn_clicked(self):
        # TODO
        raise NotImplementedError

    def on_btn_clicked(self):
        try:
            # TODO: once connexion feature is added seperately, change this line
            # check UART connexion
            self.model.set_is_connected(True)
            # start data stream
            self.model.set_is_running(True)
            # [optional) send temperature setpoint to UART
            # self.model.set_temperature(0.0)
        except Exception as e:
            print(f"Unable to start: {e}")

    def off_btn_clicked(self):
        try:
            self.model.set_is_running(False)
        except Exception as e:
            print(f"Unable to start: {e}")

    def fetch_data(self):
        # TODO
        raise NotImplementedError

    def cleanup(self):
        # TODO
        pass # we can do without for now

    

if __name__ == "__main__":
    SCENARIO = 1

    print(">> Testing controllers.py <<")

    if SCENARIO==1:
        from views import MainView
        view = MainView()
        controller = MainController(view)
    else:
        print("Nothing to test.")