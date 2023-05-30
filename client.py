import pickle
import socket
import struct
import sys
import time
from threading import *

import cv2
import numpy as np

from command import *


def start_tcp_client(ip, port):
    client = socket.socket()
    client.connect((ip, port))
    print(f'Connected to {ip}:{port}')

    return client


class Client:

    def __init__(self, ip, port=8787, video_port=8888):
        self.server_ip = ip
        self.video_port = video_port
        self.client = start_tcp_client(ip, port)
        # thread = Thread(target=self.waiting_for_message)
        # Thread test pour envoyer des messages

        self.video_client = start_tcp_client(ip, video_port)
        self.request_video()

        thread_send = Thread(target=self.send_msg)
        thread_data = Thread(target=self.data_collection(ip))

        thread_send.start()
        thread_data.start()

    def request_video(self):
        stream_bytes = b' '
        with self.video_client.makefile('rb') as connection:
            while True:
                try:
                    stream_bytes = connection.read(4)
                    frame_length = struct.unpack('<L', stream_bytes[:4])
                    jpg = connection.read(frame_length[0])
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    # showing camera live
                    # cv2.imshow('image', image)
                    # if cv2.waitKey(10) == 13:
                    #     break
                except Exception as e:
                    print(e)
                    cv2.destroyAllWindows()
                    break

    def data_collection(self, ip, timer=5):
        '''
        Open a socket and try to connect to the given IP at the port 5005
        When connected, request for the current state of the car every "timer" value in seconds
        '''
        TCP_PORT = 5005
        while True:
            try:
                client_data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_data_socket.connect((ip, TCP_PORT))
                client_data_socket.send(Command.CMD_DATA.value.encode("utf-8"))
                serialized_data = client_data_socket.recv(1024)
                if not serialized_data:
                    print("Connexion with server lost...")
                    break
                data = pickle.loads(serialized_data)
                client_data_socket.close()
                data.getData()
                time.sleep(timer)
            except socket.error as e:
                print("Connexion error :", str(e))
                print("Trying to  reconnect in 5 seconds...")
                time.sleep(5)
                continue
            except KeyboardInterrupt:
                print(str(e))
                break
            finally:
                client_data_socket.close()

    def waiting_for_message(self):
        while True:
            data = self.client.recv(1024).decode('utf-8')
            if data == '':
                break
            print(data)

    def send_msg(self):
        while True:
            txt = input('? ').encode('utf-8')
            n = self.client.send(txt)
            if n != len(txt):
                print('erreur d\'envoie')
                break

    # def send_msg(self, data):
    #     """
    #     data shape : Command.CMD_XXX.value YYY_YYY_YYY_YYY
    #     """
    #     n = self.client.send(data.encode('utf-8'))
    #     if n != len(data):
    #         print('sending error')


if __name__ == '__main__':
    client_ui = Client(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
