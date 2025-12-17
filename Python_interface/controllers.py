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
            print(f"{type(self).__name__}.bind_callbacks()")
        self.view.on_button.clicked.connect(self.on_btn_clicked)
        self.view.off_button.clicked.connect(self.off_btn_clicked)
        self.view.temperature_setpoint_inputbutton.clicked.connect(self.temperature_setpoint_btn_clicked)

    def temperature_setpoint_btn_clicked(self):
        # send setpoint to the card (through model's setpoint)
        new_setpoint = self.view.get_typed_temperature_setpoint()
        if DEBUG:
            print(f"{type(self).__name__}.setpoint_btn_clicked() - New setpoint: {new_setpoint}")
        try:
            self.model.set_temperature_setpoint(new_setpoint)
        except Exception as e:
            if DEBUG:
                print(f"{type(self).__name__}.setpoint_btn_clicked() - Unable to set new setpoint:{e}")
        # clear entrybox
        self.view.clear_temperature_setpoint_inputbox()
        # update setpoint display
        ## automatically done thanks to observer pattern

    def on_btn_clicked(self):
        try:
            # TODO once connexion feature is added seperately, change this line.
            # check UART connexion
            self.model.set_is_connected(True)
            # start data stream
            self.model.set_is_running(True)
            # [optional) send temperature setpoint to UART
            # self.model.set_temperature(0.0)
        except Exception as e:
            print(f"{type(self).__name__}.on_btn_clicked() - Unable to start: {e}")

    def off_btn_clicked(self):
        try:
            self.model.set_is_running(False)
        except Exception as e:
            print(f"{type(self).__name__}.off_btn_clicked() - Unable to stop: {e}")

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