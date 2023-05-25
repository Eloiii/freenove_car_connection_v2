import socket
from threading import *


class Client:
    """
    TODO
    - create tcp connection
    - read msg
    - perform actions
        - send motor move
        - send servos move
        - send LEDs change
        - ask for battery
        - ask for camera
    """

    def __init__(self, ip='127.0.1.1', port=8888):
        self.client = None

        self.start_tcp_client(ip, port)

    def start_tcp_client(self, ip, port):

        client = socket.socket()
        client.connect((ip, port))
        print(f'Connected to {ip}:{port}')

        self.client = client

        thread = Thread(target=self.waiting_for_message)
        # Thread test pour envoyer des messages
        thread_send = Thread(target=self.send_msg)

        thread.start()
        thread_send.start()

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
        n = self.client.send(data)
        if n != len(data):
            print('sending error')


if __name__ == '__main__':
    client_ui = Client()
