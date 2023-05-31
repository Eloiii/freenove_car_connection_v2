from flask import Flask, request, session, render_template, url_for, redirect, jsonify

from tcp_connections.client import *

app = Flask(__name__)
app.secret_key = 'SECRET_KEY'
"""
TODO

prevent choices when no IP given
another port field for the camera ?
default port ?

"""


@app.route('/', methods=('GET', 'POST'))
def hello_world():  # put application's code here
    if request.method == 'POST':
        ip = request.form['ip']
        port = request.form['port']

        session['ip'] = ip
        session['port'] = port

        client = Client()
        client.setup(session.get('ip'), int(session.get('port')))

        return redirect(url_for('choices'))

    return render_template('index.html')


@app.route('/choices')
def choices():
    return render_template('choices.html')


def open_tcp_connection_and_send(command):
    try:
        client = Client(session.get('ip'), int(session.get('port')))
        client.send_msg(command)
        client.close_connection()
    except Exception as err:
        return f"{err}"


@app.route('/toggleLed')
async def toggle_led():
    client = Client()

    value = request.args.get('value')
    client.send_msg(f'led {value}')

    data = await client.get_last_state()

    return jsonify(data.__dict__)


@app.route('/toggleBuzzer')
def toggle_buzzer():
    value = request.args.get('value')
    err = open_tcp_connection_and_send(f'buzzer {value}')

    return err if err is not None else f'Buzzer set to {value}'


@app.route('/setMotors')
def set_motors():
    value = request.args.get('value')
    err = open_tcp_connection_and_send(f'motor {value}')

    return err if err is not None else f'Motors set to {value}'


@app.route('/setServo')
def set_servo():
    value = request.args.get('value')
    err = open_tcp_connection_and_send(f'servo {value}')

    return err if err is not None else f'Servo set to {value}'


if __name__ == '__main__':
    app.run()
