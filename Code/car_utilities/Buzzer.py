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
        if command != "0":
            self.state = False
        else:
            self.state = True
        GPIO.output(Buzzer_Pin, self.state)

    def isOn(self):
        return self.state


if __name__ == '__main__':
    B = Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')
