from flask import Flask, request
from client import *
from command import *

app = Flask(__name__)


@app.route('/')
def hello_world():  # put application's code here
    return 'Hello World!'


@app.route('/toggleLed')
def toggle_led():
    client = Client('138.250.156.7', 8785, 8888)
    value = request.args.get('value')
    client.send_msg(f'led {value}')
    client.close_connection()

    return f'LED set to {value}'


@app.route('/toggleBuzzer')
def toggle_buzzer():
    client = Client('138.250.156.7', 8785, 8888)
    value = request.args.get('value')
    client.send_msg(f'buzzer {value}')
    client.close_connection()

    return f'LED set to {value}'


if __name__ == '__main__':
    app.run(host='0.0.0.0')
