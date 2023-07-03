import RPi.GPIO as GPIO

from .Motor import *


class Line_Tracking:
    def __init__(self):
        self.IR01 = 14
        self.IR02 = 15
        self.IR03 = 23
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.IR01, GPIO.IN)
        GPIO.setup(self.IR02, GPIO.IN)
        GPIO.setup(self.IR03, GPIO.IN)
        self.running = False

    def stop(self):
        self.running = False

    def run(self):
        if self.running:
            return
        self.running = True
        while self.running:
            self.LMR = 0x00
            if GPIO.input(self.IR01) == True:
                self.LMR = (self.LMR | 4)
            if GPIO.input(self.IR02) == True:
                self.LMR = (self.LMR | 2)
            if GPIO.input(self.IR03) == True:
                self.LMR = (self.LMR | 1)
            if self.LMR == 2:
                PWM.setMotorModel(800, 800, 800, 800)
            elif self.LMR == 4:
                PWM.setMotorModel(-1500, -1500, 2500, 2500)
            elif self.LMR == 6:
                PWM.setMotorModel(-2000, -2000, 3500, 4000)
            elif self.LMR == 1:
                PWM.setMotorModel(2500, 2500, -1500, -1500)
            elif self.LMR == 3:
                PWM.setMotorModel(4000, 4000, -2000, -2000)
            elif self.LMR == 7:
                # pass
                PWM.setMotorModel(0, 0, 0, 0)