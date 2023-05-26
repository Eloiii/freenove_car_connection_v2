import fcntl
import io
import socket
import struct
import sys
from threading import *

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.encoders import Quality
from picamera2.outputs import FileOutput

from car_utilities.Buzzer import *
from car_utilities.Led import *
from car_utilities.Light import *
from car_utilities.Ultrasonic import *
from command import *


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def start_tcp_server(port):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # retrieve IP address linked to the 'wlan0' interface
    ip_addr = socket.inet_ntoa(fcntl.ioctl(server.fileno(),
                                           0x8915,
                                           struct.pack('256s', b'wlan0'[:15])
                                           )[20:24])
    server.bind((ip_addr, port))
    server.listen(1)
    print(f"Server up, listening on {ip_addr}:{port}")
    return server


class Server:

    def __init__(self, port=8787, video_port=8888):
        self.server = start_tcp_server(port)
        thread = Thread(target=self.waiting_for_connection)
        thread.start()

        self.video_server = None
        self.video_port = video_port

        self.motor_manager = Motor()
        self.servo_manager = Servo()
        self.led_manager = Led()
        self.buzzer_manager = Buzzer()
        self.adc = Adc()

    def start_video_server(self):
        self.video_server = start_tcp_server(self.video_port)
        self.video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        thread = Thread(target=self.waiting_for_camera_connection)
        thread.start()

    def waiting_for_connection(self):
        while True:
            client, client_addr = self.server.accept()
            print("New connection from ", client_addr)
            thread = Thread(target=self.receive_data, args=(client, client_addr))
            thread.start()

    def receive_data(self, client, client_addr):
        while True:
            data = client.recv(1024).decode('utf-8')
            """
            TODO 
            if client disconnect, wait for another, don't crash the server
            """
            if not data:
                break
            else:
                self.treat_msg(data)
            print(data)
        client.close()

    def waiting_for_camera_connection(self):

        connection, client_address = self.video_server.accept()
        connection = connection.makefile('wb')

        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (400, 300)}))
        output = StreamingOutput()
        encoder = MJPEGEncoder(10000000)
        camera.start_recording(encoder, FileOutput(output), quality=Quality.VERY_HIGH)
        while True:
            with output.condition:
                output.condition.wait()
                frame = output.frame
            try:
                frame_length = len(output.frame)
                frame_length_binary = struct.pack('<I', frame_length)
                connection.write(frame_length_binary)
                connection.write(frame)
            except:
                camera.stop_recording()
                camera.close()
                print("End transmit ... ")
                break

    """
    msg shape : Command.CMD_XXX.value YYY_YYY_YYY_YYY
    """

    def treat_msg(self, msg):
        split_msg = msg.split(' ')
        cmd = split_msg[0]

        if cmd == Command.CMD_MOTOR.value:
            # motor 2000_2000_2000_2000
            self.activate_motor(split_msg[1])
        elif cmd == Command.CMD_SERVO.value:
            # servo 0_90
            self.activate_servo(split_msg[1])
        elif cmd == Command.CMD_LED.value:
            # led 0x01_255_255_255
            self.activate_led(split_msg[1])
        elif cmd == Command.CMD_SONIC.value:
            # sonic
            self.send_ultrasonic()
        elif cmd == Command.CMD_BUZZER.value:
            # buzzer 1
            self.activate_buzzer(split_msg[1])
        elif cmd == Command.CMD_LIGHT.value:
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
        int_converted = list(map(lambda n: int(n, 0), params))
        self.led_manager.ledIndex(*int_converted)

    def send_ultrasonic(self, param):
        # _
        pass

    def activate_buzzer(self, param):
        self.buzzer_manager.run(param)


if __name__ == '__main__':
    Server(int(sys.argv[1]), int(sys.argv[2]))
