import socket
import sys
import pickle
from threading import *
from command import *
from car_utilities.DataCollection import *


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
        thread_data = Thread(target=self.data_collection(ip))

        thread.start()
        thread_send.start()
        thread_data.start()



    def data_collection(self, ip):
        TCP_PORT = 5005 
        while True:
            try:
                client_data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                client_data_socket.connect((ip, TCP_PORT))
                while True:
                    serialized_data = client_data_socket.recv(1024)
                    if not serialized_data:
                        print("Connexion with server lost...")
                        break
                    data = pickle.loads(serialized_data)
                    data.getData()
                    time.sleep(15)

            except socket.error as e:
                # Gérer les erreurs de connexion
                print("Erreur de connexion :", str(e))
                print("Tentative de reconnexion dans 5 secondes...")
                time.sleep(5)
                continue

            except KeyboardInterrupt:
                # Gérer l'interruption par l'utilisateur
                print("Arrêt de l'écoute.")
                break

            finally:
                # Fermer la socket
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

    #def send_msg(self, data):
    #    """
    #    data shape : Command.CMD_XXX.value YYY_YYY_YYY_YYY
    #    """
    #    n = self.client.send(data.encode('utf-8'))
    #    if n != len(data):
    #        print('sending error')


if __name__ == '__main__':
    client_ui = Client(sys.argv[1], int(sys.argv[2]))
