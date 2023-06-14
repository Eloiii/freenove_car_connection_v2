import pickle
import socket
import struct
from threading import *

import cv2
import numpy as np

from tcp_connections.car_utilities.camera_data import *
from tcp_connections.command import *

from database.db import *
sys.path.append('..')


def start_tcp_client(ip, port):
    client = socket.socket()
    client.connect((ip, port))
    print(f'Connected to {ip}:{port}')
    return client


class ClientMeta(type):
    _instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            instance = super().__call__(*args, **kwargs)
            cls._instances[cls] = instance
        return cls._instances[cls]


class Client(metaclass=ClientMeta):

    def __init__(self):
        self.timer = None
        self.data_collection_bool = None
        self.last_state = None
        self.video_client = None
        self.client = None
        self.video_port = None
        self.server_ip = None
        self.img_bytes = None
        self.initialised = False

        self.thread_bool = True

    def __new__(cls, *args, **kwargs):
        if not hasattr(cls, 'instance'):
            cls.instance = super(Client, cls).__new__(cls)
        return cls.instance

    def setup(self, ip, port=Port.PORT_COMMAND.value, video_port=Port.PORT_VIDEO.value, data_port=Port.PORT_DATA.value):
        self.server_ip = ip
        self.video_port = video_port
        self.client = start_tcp_client(ip, port)
        self.video_client = None
        self.last_state = None
        self.data_collection_bool = True

        self.timer = 1
        thread_data = Thread(target=self.data_collection, args=(ip, data_port,))
        thread_data.start()
        self.initialised = True

    def connect_to_video_server(self, framerate, width, height, save_images=True):
        self.video_client = start_tcp_client(self.server_ip, self.video_port)
        camera_data = Camera_data()
        camera_data.framerate = framerate
        camera_data.width = width
        camera_data.height = height
        self.video_client.send(pickle.dumps(camera_data))
        thread_video = Thread(target=self.start_recording, args=(save_images,))
        thread_video.start()

    def start_recording(self, save_images=True):
        n_img = 0
        stream_bytes = b' '
        if save_images:
            directory = 'images_' + str(datetime.now()).replace(' ', '_')
            os.mkdir(f'./{directory}')
        with self.video_client.makefile('rb') as connection:
            while True:
                try:
                    stream_bytes = connection.read(4)
                    frame_length = struct.unpack('<L', stream_bytes[:4])
                    self.img_bytes = connection.read(frame_length[0])
                    image = cv2.imdecode(np.frombuffer(self.img_bytes, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if save_images:
                        cv2.imwrite(f'{directory}/{n_img}.jpg', image)
                    n_img += 1
                    # showing camera live
                    # cv2.imshow('image', image)
                    # if cv2.waitKey(10) == 13:
                    #     break
                except Exception as e:
                    print(e)
                    cv2.destroyAllWindows()
                    break

    def data_collection(self, ip, port):
        """
        Open a socket and try to connect to the given IP at the port 5005
        When connected, request for the current state of the car every "timer" value in seconds
        """
        onto = start_database()
        while self.thread_bool:
            self.client_data_socket = start_tcp_client(ip, port)
            try:
                while self.thread_bool:
                    self.client_data_socket.send(Command.CMD_DATA.value.encode("utf-8"))
                    serialized_data = self.client_data_socket.recv(1024)
                    if not serialized_data:
                        print("Connexion with server lost...")
                        break
                    data = pickle.loads(serialized_data)
                    self.last_state = data
                    if (self.data_collection_bool):
                        add_car_data_to_db(data=data, onto=onto)
                        default_world.save()
                    # print_data(self.last_state)
                    time.sleep(self.timer)
            except socket.error as e:
                print("Connexion error :", str(e))
                print("Trying to reconnect in 5 seconds...")
                time.sleep(5)
                continue

    def send_msg(self, data):
        """
        data shape : Command.CMD_XXX.value YYY_YYY_YYY_YYY
        """
        n = self.client.send(data.encode('utf-8'))
        if n != len(data):
            print('sending error')

    def close_data_connection(self):
        self.client_data_socket.shutdown(socket.SHUT_RDWR)
        self.client_data_socket.close()

    def close_video_connection(self):
        self.video_client.shutdown(socket.SHUT_RDWR)
        self.video_client.close()

    def close_connection(self):
        self.client.shutdown(socket.SHUT_RDWR)
        self.client.close()

    async def get_last_state(self):
        while self.last_state is None:
            continue
        return self.last_state


if __name__ == '__main__':
    client_ui = Client()
    client_ui.setup(sys.argv[1])
