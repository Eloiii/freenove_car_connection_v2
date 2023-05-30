import fcntl
import io
import socket
import struct
import sys
import pickle
import psutil
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
from car_utilities.DataCollection import *


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

    def __init__(self, port=8787, video_port=8888, data_port=8666):
        self.server = start_tcp_server(port)
        self.data_socket = start_tcp_server(data_port)

        self.motor_manager = Motor()
        self.servo_manager = Servo()
        self.led_manager = Led()
        self.buzzer_manager = Buzzer()
        self.adc = Adc()

        self.data = Data()

        self.video_server = start_tcp_server(video_port)
        self.video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        data_thread = Thread(target=self.data_collection)
        data_thread.start()
        thread = Thread(target=self.waiting_for_connection)
        thread.start()
        thread_camera = Thread(target=self.waiting_for_camera_connection)
        thread_camera.start()

    def waiting_for_connection(self):
        try:
            while True:
                client, client_addr = self.server.accept()
                print("New connection from ", client_addr)
                thread = Thread(target=self.receive_data, args=(client, client_addr))
                thread.start()
        except KeyboardInterrupt:
            print("[!] Keyboard Interrupted!")
            self.close_server()

    def close_server(self):
        self.server.shutdown(socket.SHUT_RDWR)
        self.server.close()

    def receive_data(self, client, client_addr):
        while True:
            data = client.recv(1024).decode('utf-8')
            if not data:
                break
            else:
                self.treat_msg(data)
            print(f'received {data}')
        client.close()
        print(f'Client {client_addr} disconnected')

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

    def data_collection(self):
        """
        When the data_socket is open.
        This function put the socket in listening mode for a client to connect.
        When the client is connected, the socket wait for it to send a request for the data of the car and send them to him when he request them
        using pickle serialization.
        """
        while True:
            client, client_addr = self.data_socket.accept()
            while True:
                try:
                    data = client.recv(1024).decode('utf-8')
                    if not data:
                        print("Connexion with client lost on data socket lost :", client_addr)
                        break
                    if data == Command.CMD_DATA.value:
                        self.data.setData(battery_voltage=self.adc.recvADC(2) * 3,
                                          battery_percent=float((self.adc.recvADC(2) * 3) - 7) / 1.40 * 100,
                                          # cap = cv2.VideoCapture(0)
                                          isRecording=False,  # cap.isOpened(),
                                          width=None,  # cap.get(cv2.CAP_PROP_FRAME_WIDTH),
                                          height=None,  # cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                                          FPS=None,  # cap.get(cv2.CAP_PROP_FPS),
                                          CPU=psutil.cpu_percent(),
                                          motor_model=self.motor_manager.getMotorModel(),
                                          leds=self.led_manager.ledsState())

                        client.send(pickle.dumps(self.data))
                    else:
                        print("Invalid request :", str(data))

                except socket.error as e:
                    print("Data socket error", client_addr, ":", str(e))
                    break

                except KeyboardInterrupt:
                    print("Data socket closing...")
                    break
            client.close()

    """
    msg shape : XXXX YYY_YYY_YYY_YYY
    XXXX a value from Command Enum
    YYYY_YYYY... parameters for the command XXXX
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
            self.sonic_count += 1
        elif cmd == Command.CMD_BUZZER.value:
            # buzzer 1
            self.activate_buzzer(split_msg[1])
        elif cmd == Command.CMD_LIGHT.value:
            # ??
            pass
        else:
            print(f'Error, unknown command {cmd}')

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
        self.data.buzz()


if __name__ == '__main__':
    Server(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
