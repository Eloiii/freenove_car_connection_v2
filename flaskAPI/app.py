from flask import Flask, request

from client import *

app = Flask(__name__)


@app.route('/')
def hello_world():  # put application's code here
    return 'try /toggleBuzzer?value=1'


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

    return f'Buzzer set to {value}'


if __name__ == '__main__':
    app.run()
