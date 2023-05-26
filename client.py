import socket
import sys
from threading import *
from command import *


class Client:

    def __init__(self, ip, port=8787, video_port=8888):
        self.client = None
        self.video_client = None
        self.server_ip = ip
        self.video_port = video_port
        self.start_tcp_client(ip, port)
        self.start_udp_client()

    def start_tcp_client(self, ip, port):

        client = socket.socket()
        client.connect((ip, port))
        print(f'Connected to {ip}:{port}')

        self.client = client

        # thread = Thread(target=self.waiting_for_message)
        # Thread test pour envoyer des messages
        # thread_send = Thread(target=self.send_msg)

        # thread.start()
        # thread_send.start()

    def start_udp_client(self):
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.makefile("rb")
        self.video_client = client
        self.request_video()

    def request_video(self):
        self.video_client.sendto('request_video'.encode(), (self.server_ip, self.video_port))
        while True:
            data, addr_serv = self.video_client.recvfrom(1024)
            print(data)

    def waiting_for_message(self):
        while True:
            data = self.client.recv(1024).decode('utf-8')
            if data == '':
                break
            print(data)

    # def send_msg(self):
    #     while True:
    #         txt = input('? ').encode('utf-8')
    #         n = self.client.send(txt)
    #         if n != len(txt):
    #             print('erreur d\'envoie')
    #             break

    def send_msg(self, data):
        """
        data shape : Command.CMD_XXX.value YYY_YYY_YYY_YYY
        """
        n = self.client.send(data.encode('utf-8'))
        if n != len(data):
            print('sending error')


if __name__ == '__main__':
    client_ui = Client(sys.argv[1], int(sys.argv[2]), int(sys.argv[3]))
