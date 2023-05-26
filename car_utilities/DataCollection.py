from datetime import datetime

class Data:

    def __init__(self):
        #Data
        self.timestamp = None

        self.battery_percent = None
        self.battery_voltage = None
        self.camera_is_recording = False
        self.camera_resolution_height = None
        self.camera_resolution_width = None
        self.camera_framerate = None
        self.CPU_use_percent = None
        self.motor_model = None
        self.leds_state= None
        self.sonic_count = 0

        


    def buzz(self):
        self.sonic_count +=1
    def reset_buzz_count(self):
        self.sonic_count = 0

    def setData(self,battery_voltage,battery_percent,CPU,motor_model,leds):
        '''
        Used by the server to update the current state of the car in the Data object
        '''
        self.timestamp = datetime.timestamp(datetime.now())

        self.battery_voltage = battery_voltage
        self.battery_percent = battery_percent
        #CAMERA DATA



        self.CPU_use_percent = CPU
        self.motor_model = motor_model
        self.leds_state = leds

        pass

    def getData(self):
        '''
        Used by the client to save the data into the Database
        '''
        print(f'CPU :{self.CPU_use_percent}%\nWheels:\nFR:{self.motor_model[0]}|FL:{self.motor_model[1]}|BR:{self.motor_model[2]}|BL:{self.motor_model[3]}\nLedsBrightness:{self.leds_state}\nSonicCount:{self.sonic_count}')
        pass