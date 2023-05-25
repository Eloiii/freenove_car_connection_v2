from flask import Flask
from client import *
from command import *

app = Flask(__name__)


@app.route('/')
def hello_world():  # put application's code here
    return 'Hello World!'


@app.route('/toggleBuzzer')
def toggle_buzzer():
    """
    two ways
    """

    # 1
    client = Client(ip, port, Command.CMD_BUZZER.value)

    # 2
    client = Client(ip, port)
    client.toggle_buzzer()


if __name__ == '__main__':
    app.run()
