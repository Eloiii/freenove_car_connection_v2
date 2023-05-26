import psutil
from datetime import datetime
from car_utilities.Motor import *
from car_utilities.Led import *
from car_utilities.Buzzer import *
from car_utilities.ADC import *


class Data:

    def __init__(self, motor: Motor, led: Led, buzzer: Buzzer, adc: Adc):
        #Objects
        self.motor = motor
        self.led = led
        self.buzzer = buzzer
        self.adc = adc


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

    def setData(self):
        '''
        Used by the server to update the current state of the car in the Data object
        '''
        self.timestamp = datetime.timestamp(datetime.now())
        self.battery_voltage = self.adc.recvADC(2)*3
        self.battery_percent = (float(self.battery_voltage)-7)/1.40*100
        #CAMERA DATA



        self.CPU_use_percent = psutil.cpu_percent()
        self.motor_model = self.motor.getMotorModel #(FR/FL/BR/BL)
        self.leds_state = self.led.ledsState()

        pass

    def getData(self):
        '''
        Used by the client to save the data into the Database
        '''
        print(f'CPU :{self.CPU_use_percent}%\nWheels:\nFR:{self.motor_model[0]}|FL:{self.motor_model[1]}|BR:{self.motor_model[2]}|BL:{self.motor_model[3]}\nLedsBrightness:{self.leds_state}\nSonicCount:{self.sonic_count}')
        pass