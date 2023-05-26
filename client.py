import io
import socket
import struct
import sys
from threading import *

import cv2
import numpy as np
from PIL import Image


def start_tcp_client(ip, port):

    client = socket.socket()
    client.connect((ip, port))
    print(f'Connected to {ip}:{port}')

    return client


def IsValidImage4Bytes(buf):
    bValid = True
    if buf[6:10] in (b'JFIF', b'Exif'):
        if not buf.rstrip(b'\0\r\n').endswith(b'\xff\xd9'):
            bValid = False
    else:
        try:
            Image.open(io.BytesIO(buf)).verify()
        except:
            bValid = False
    return bValid


class Client:

    def __init__(self, ip, port=8787, video_port=8888):
        self.server_ip = ip
        self.video_port = video_port
        self.client = start_tcp_client(ip, port)
        # thread = Thread(target=self.waiting_for_message)
        # Thread test pour envoyer des messages
        thread_send = Thread(target=self.send_msg)
        # thread.start()
        thread_send.start()

        self.video_client = start_tcp_client(ip, video_port)
        self.request_video(ip, video_port)

    def request_video(self, ip, port):
        stream_bytes = b' '
        with self.video_client.makefile('rb') as connection:
            while True:
                try:
                    stream_bytes = connection.read(4)
                    leng = struct.unpack('<L', stream_bytes[:4])
                    jpg = connection.read(leng[0])
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    cv2.imshow('image', image)
                    if cv2.waitKey(10) == 13:
                        break
                except Exception as e:
                    print(e)
                    cv2.destroyAllWindows()
                    break

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
