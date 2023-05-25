import socket
from threading import *
from enum import Enum
from car_utilities.Led import *
from car_utilities.Buzzer import * 
from car_utilities.Light import *
from car_utilities.Ultrasonic import *


class Command(Enum):
    CMD_MOTOR = 'motor'
    CMD_SERVO = 'servo'
    CMD_LED = 'led'
    CMD_SONIC = 'sonic'
    CMD_BUZZER = 'buzzer'
    CMD_LIGHT = 'light'


class Server:
    """
    TODO
    - create tcp connection
    - read msg
    - perform actions
        - move motors
        - move servos
        - change LEDs
        - manage ultrasonic
        - send power data
    """

    def __init__(self, port=8888):
        self.server = None
        self.start_tcp_server(port)
        self.motor_manager = Motor()
        self.servo_manager = Servo()
        self.led_manager = Led()
        self.buzzer_manager = Buzzer()
        self.adc = Adc()

    def start_tcp_server(self, port):
        hostname = socket.gethostname()
        ip_addr = socket.gethostbyname(hostname)

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.bind((ip_addr, port))
        server.listen(1)

        self.server = server

        print(f"Server up, listening on {ip_addr}:{port}")

        thread = Thread(target=self.waiting_for_connection)
        thread.start()

    def waiting_for_connection(self):
        client, client_addr = self.server.accept()
        print("New connection from ", client_addr)

        while True:
            data = client.recv(1024).decode('utf-8')
            if not data:
                break
            else:
                self.treat_msg(data)
            print(data)

    """
    :param msg is of the form : Command XXX_XXX
    """
    def treat_msg(self, msg):
        split_msg = msg.split(' ')
        cmd = split_msg[0]
        if cmd == Command.CMD_MOTOR:
            # motor 2000_2000_2000_2000
            self.activate_motor(split_msg[1])
        elif cmd == Command.CMD_SERVO:
            # servo 0_90
            self.activate_servo(split_msg[1])
        elif cmd == Command.CMD_LED:
            # led 0x01_255_255_255
            self.activate_led(split_msg[1])
        elif cmd == Command.CMD_SONIC:
            # sonic
            self.send_ultrasonic()
        elif cmd == Command.CMD_BUZZER:
            # buzzer 1
            self.activate_buzzer(split_msg[1])
        elif cmd == Command.CMD_LIGHT:
            # ??
            pass
        else:
            print('Error, unknown command')

    def activate_motor(self, param):
        # 2000_2000_2000_2000
        pwm_values = param.split('_')
        int_converted = list(map(lambda n: int(n), pwm_values))
        self.motor_manager.setMotorModel(*int_converted)

    def activate_servo(self, param):
        # 0_90
        params = param.split('_')
        self.servo_manager.setServoPwm(params[0], int(params[1]))

    def activate_led(self, param):
        # 0x01_255_255_255
        params = param.split('_')
        int_converted = list(map(lambda n: int(n), params))
        self.led_manager.ledIndex(*int_converted)

    def send_ultrasonic(self, param):
        # _
        pass

    def activate_buzzer(self, param):
        self.buzzer_manager.run(int(param))


if __name__ == '__main__':
    Server()
