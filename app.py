from flask import Flask, request, session, render_template, url_for, redirect, jsonify

from tcp_connections.client import *

app = Flask(__name__)
app.secret_key = 'SECRET_KEY'


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


async def send_msg_and_receive_state(command):
    client = Client()

    client.send_msg(command)

    data = await client.get_last_state()

    return data


@app.route('/toggleLed')
async def toggle_led():
    value = request.args.get('value')
    state = await send_msg_and_receive_state(f'led {value}')

    return jsonify(state.__dict__)


@app.route('/toggleBuzzer')
async def toggle_buzzer():
    value = request.args.get('value')
    state = await send_msg_and_receive_state(f'buzzer {value}')

    return jsonify(state.__dict__)


@app.route('/setMotors')
async def set_motors():
    value = request.args.get('value')
    state = await send_msg_and_receive_state(f'motor {value}')

    return jsonify(state.__dict__)


@app.route('/setServo')
async def set_servo():
    value = request.args.get('value')
    state = await send_msg_and_receive_state(f'servo {value}')

    return jsonify(state.__dict__)


if __name__ == '__main__':
    app.run()
