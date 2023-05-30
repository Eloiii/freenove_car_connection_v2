from datetime import datetime

def new_ID():

    pass

class Data:

    def __init__(self):
        self.Car_MAC = None         #
        self.Car_ip_address = None  #
        self.samplin_rate = None    #
        #Data
        self.timestamp = None
        self.battery_percent = None
        self.battery_voltage = None
        self.camera_is_recording = None
        self.camera_resolution_height = None
        self.camera_resolution_width = None
        self.camera_framerate = None
        self.CPU_use_percent = None
        self.nb_process = None #
        self.motor_model = None #A list of the use of each wheel FR|FL|BR|BL
        self.leds_state= None # A list of RGB code for each Leds
        self.ultrasonic_inUse = None #TO ADD
        self.sonic_count = 0
        
    def buzz(self):
        self.sonic_count += 1

    def reset_buzz_count(self):
        self.sonic_count = 0

    def setData(self,MAC:str=None,IP:str=None,battery_voltage:float=None,battery_percent:float=None,isRecording:bool=None,height=None,width=None,FPS=None,CPU:float=None,nb_process:int=None,motor_model=None,leds=None, ultrasonic=None):
        '''
        Used by the server to update the current state of the car in the Data object
        '''
        self.timestamp = datetime.timestamp(datetime.now())
        self.Car_MAC = MAC
        self.Car_ip_address = IP
        self.battery_voltage = battery_voltage
        self.battery_percent = battery_percent
        self.camera_is_recording = isRecording
        self.camera_resolution_height = height
        self.camera_resolution_width = width
        self.camera_framerate = FPS
        self.CPU_use_percent = CPU
        self.nb_process = nb_process
        self.motor_model = motor_model
        self.leds_state = leds
        self.Ultrasonic_inUse = ultrasonic

    def getData(self, samplin_rate:int=None):
        '''
        Used by the client to save the data into the Database
        Furthermore update the sampling rate
        '''
        self.samplin_rate=samplin_rate
        print(f'CPU :{self.CPU_use_percent}%\nWheels:\nFR:{self.motor_model[0]}|FL:{self.motor_model[1]}|BR:{self.motor_model[2]}|BL:{self.motor_model[3]}\nLedsBrightness:{self.leds_state}\nSonicCount:{self.sonic_count}')
        pass
