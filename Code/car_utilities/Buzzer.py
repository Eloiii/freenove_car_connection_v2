import time

import RPi.GPIO as GPIO

GPIO.setwarnings(False)
Buzzer_Pin = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(Buzzer_Pin, GPIO.OUT)


class Buzzer:
    def __init__(self):
        self.state = None

    def run(self, command):
        self.state = command
        if command != "0":
            State = False
        else:
            State = True
        GPIO.output(Buzzer_Pin, State)

    def isOn(self):
        return self.state


if __name__ == '__main__':
    B = Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')
