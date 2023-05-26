import socket
import struct
import fcntl
import sys
import pickle
from threading import *
from command import *
from car_utilities.Led import *
from car_utilities.Buzzer import *
from car_utilities.Light import *
from car_utilities.Ultrasonic import *
from car_utilities.DataCollection import *


class Server:

    def __init__(self, port=8787, data_port=5005):

        self.start_tcp_server(port, data_port)
        self.motor_manager = Motor()
        self.servo_manager = Servo()
        self.led_manager = Led()
        self.buzzer_manager = Buzzer()
        self.adc = Adc()

        self.data = Data(motor=self.motor_manager, led=self.led_manager, buzzer=self.buzzer_manager, adc=self.adc)

       


    def start_tcp_server(self, port, data_port):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        ip_addr = socket.inet_ntoa(fcntl.ioctl(self.server_socket.fileno(),
                                               0x8915,
                                               struct.pack('256s', b'wlan0'[:15])
                                               )[20:24])
        self.server_socket.bind((ip_addr, port))
        self.server_socket.listen(1)
        self.data_socket.bind((ip_addr, data_port))
        self.data_socket.listen(1)


        print(f"Server up, listening on {ip_addr}:{port}")
        
        thread = Thread(target=self.waiting_for_connection)
        thread.start()
        data_thread = Thread(target=self.data_collection)
        thread.start()


    def waiting_for_connection(self):
        client, client_addr = self.server_socket.accept()
        print("New connection from ", client_addr)
        while True:
            data = client.recv(1024).decode('utf-8')
            if not data:
                break
            else:
                self.treat_msg(data)
            print(data)


    def data_collection(self):
        while True:
            # Accepter une nouvelle connexion
            client, client_addr = self.data_socket.accept()

            while True:
                try:
                    data = client.recv(1024).decode('utf-8')
                    if not data:
                        print("Connexion with client lost on data socket lost :", client_addr)
                        break
                    if(data==Command.CMD_DATA.value):
                        self.data.setData()
                        client.send(pickle.dumps(self.data))
                    else:
                        print("Invalid request :", data)

                except socket.error as e:
                    print("Data socket error", client_addr, ":", str(e))
                    break

                except KeyboardInterrupt:
                    print("Data socket closing...")
                    break
            client.close()
        self.data_socket.close()



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
            self.sonic_count += 1
        elif cmd == Command.CMD_BUZZER.value:
            # buzzer 1
            self.activate_buzzer(split_msg[1])
        elif cmd == Command.CMD_LIGHT.value:
            # ??
            pass
        else:
            print('Error, unknown command')
        self.get_State()

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
    Server(int(sys.argv[1]))
