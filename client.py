import socket
import sys
from threading import *
from command import *


class Client:

    def __init__(self, ip, port=8888):
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

    def send_msg(self):
        while True:
            txt = input('? ').encode('utf-8')
            n = self.client.send(txt)
            if n != len(txt):
                print('erreur d\'envoie')
                break

    #def send_msg(self, data):
    #    """
    #    data shape : Command.CMD_XXX.value YYY_YYY_YYY_YYY
    #    """
    #    n = self.client.send(data.encode('utf-8'))
    #    if n != len(data):
    #        print('sending error')


if __name__ == '__main__':
    client_ui = Client(sys.argv[1], int(sys.argv[2]))
