import fcntl
import io
import pickle
import socket
import struct
import uuid
import psutil
from threading import *

from picamera2 import Picamera2
from picamera2.encoders import MJPEGEncoder
from picamera2.encoders import Quality
from picamera2.outputs import FileOutput

from Code.car_utilities.Buzzer import *
from Code.car_utilities.Led import *
from Code.car_utilities.Light import *
from Code.car_utilities.Ultrasonic import *
from Code.car_utilities.Line_Tracking import *
from Code.data.packet import *
from Code.enumerate import *


def class_threading(func):
    def wrapper(self, *args, **kwargs):
        thread = Thread(target=func, args=(self, *args,), kwargs=kwargs)
        thread.start()

    return wrapper


def get_mac_address():
    mac = uuid.getnode()
    mac_address = ':'.join(("%012X" % mac)[i:i + 2] for i in range(0, 12, 2))
    return mac_address


class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


def start_tcp_server(port):
    """
    Open a TCP server socket at the given port and bind it to the machine IP
    Return the server
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # retrieve IP address linked to the 'wlan0' interface
    ip_addr = socket.inet_ntoa(fcntl.ioctl(server.fileno(),
                                           0x8915,
                                           struct.pack('256s', b'wlan0'[:15])
                                           )[20:24])
    server.bind((ip_addr, port))
    server.listen(1)
    return server


class Server:

    def __init__(self, port=Port.PORT_COMMAND.value, video_port=Port.PORT_VIDEO.value, data_port=Port.PORT_DATA.value):
        self.motor_manager = PWM
        self.servo_manager = Servo()
        self.led_manager = Led()
        self.buzzer_manager = Buzzer()
        self.line_tracking = Line_Tracking()
        self.adc = Adc()
        self.data = Data()
        self.sonic = False
        self.camera_is_recording = False
        self.camera_height = 300
        self.camera_width = 400
        self.camera_framerate = 25

        self.server = start_tcp_server(port)
        print(f"Command server up, listening on {port}")

        self.data_server = start_tcp_server(data_port)
        print(f"data server up, listening on {data_port}")

        self.video_server = start_tcp_server(video_port)
        self.video_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print(f"Video server up, listening on {video_port}")

        # Set up all the daemon threads
        self.data_collection()
        self.waiting_for_connection()
        self.waiting_for_camera_connection()

    @class_threading
    def waiting_for_connection(self):
        while True:
            client, client_addr = self.server.accept()
            print("New connection from ", client_addr)
            try:
                self.receive_data(client, client_addr)
            except:
                break
        self.close_server()

    def close_server(self):
        self.server.shutdown(socket.SHUT_RDWR)
        self.server.close()

    @class_threading
    def receive_data(self, client, client_addr):
        while True:
            data = client.recv(1024).decode('utf-8')
            if not data:
                break
            else:
                try:
                    self.treat_msg(data)
                except:
                    client.close()
                    self.close_server()
                    return
            print(f'received {data}')
        client.close()
        print(f'Client {client_addr} disconnected')

    @class_threading
    def waiting_for_camera_connection(self):
        while True:
            connection, client_address = self.video_server.accept()

            raw_camera_data = connection.recv(1024)
            camera_data = pickle.loads(raw_camera_data)

            connection = connection.makefile('wb')

            self.record_and_send_video(connection, camera_data.framerate, camera_data.width, camera_data.height)

    @class_threading
    def record_and_send_video(self, connection, framerate, resolution_width, resolution_height):

        if framerate is not None:
            self.camera_framerate = int(framerate)
        if resolution_width is not None:
            self.camera_width = int(resolution_width)
        if resolution_height is not None:
            self.camera_height = int(resolution_height)

        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (self.camera_width, self.camera_height)}))
        output = StreamingOutput()
        encoder = MJPEGEncoder(10000000)
        camera.start_recording(encoder, FileOutput(output), quality=Quality.VERY_HIGH)
        self.camera_is_recording = True
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
                self.camera_is_recording = False
                print("End transmit ... ")
                break
            time.sleep(1 / self.camera_framerate)

    @class_threading
    def data_collection(self):
        """
        When the data_server is open. This function put the socket in listening mode for a client to connect. When
        the client is connected, the socket wait for it to send a request for the data of the car and send them to
        him when it requests them using pickle serialization.
        """
        while True:
            client, client_addr = self.data_server.accept()
            while True:
                try:
                    data = client.recv(1024).decode('utf-8')
                    if not data:
                        print("Connexion with client lost on data socket lost :", client_addr)
                        break
                    if data == Command.CMD_DATA.value:
                        set_data(data=self.data,
                                 MAC=get_mac_address(),
                                 IP=None,
                                 battery_voltage=self.adc.recvADC(2) * 3,
                                 battery_percent=((self.adc.recvADC(2) * 3) - 7) / 1.40 * 100,
                                 isRecording=self.camera_is_recording,
                                 width=self.camera_width,
                                 height=self.camera_height,
                                 FPS=self.camera_framerate,
                                 CPU=psutil.cpu_percent(),
                                 nb_process=len(list(psutil.process_iter())),
                                 motor_model=self.motor_manager.getMotorModel(),
                                 leds=self.led_manager.ledsState(),
                                 ultrasonic=self.sonic,
                                 buzzer=self.buzzer_manager.isOn())
                        client.send(pickle.dumps(self.data))
                    else:
                        print("Invalid request :", str(data))

                except socket.error as e:
                    print("data socket error", client_addr, ":", str(e))
                    break

                except KeyboardInterrupt:
                    print("data socket closing...")
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
        try:
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
                # sonic TODO
                self.send_ultrasonic()
            elif cmd == Command.CMD_BUZZER.value:
                # buzzer 1
                self.activate_buzzer(split_msg[1])
            elif cmd == Command.CMD_LINE_TRACKING.value:
                line_tracking_thread = Thread(target=self.line_tracking.run)
                line_tracking_thread.start()
            elif cmd == Command.STOP_CMD_LINE_TRACKING.value:
                self.line_tracking.stop()
            else:
                print(f'Error, unknown command {cmd}')
        except Exception as e:
            print(f'An error has occured while processing the command {cmd}')
            print(e)
            return

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
    Server()
