import time
import RPi.GPIO as GPIO

GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin,GPIO.OUT)
State = False

class Buzzer:
    def run(self,command):
        if command!="0":
            State = False
        else:
            State = True
        GPIO.output(Buzzer_Pin,State)
    def isOn(self):
        return State


if __name__=='__main__':
    B=Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')




