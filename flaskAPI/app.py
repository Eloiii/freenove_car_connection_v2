import json

from flask import Flask, request, session, render_template, url_for, redirect, jsonify

from client import *

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


@app.route('/toggleLed')
async def toggle_led():

    client = Client()

    value = request.args.get('value')
    client.send_msg(f'led {value}')

    data = await client.get_last_state()

    return jsonify(data.__dict__)


@app.route('/toggleBuzzer')
async def toggle_buzzer():
    client = Client(session.get('ip'), int(session.get('port')))
    value = request.args.get('value')
    client.send_msg(f'buzzer {value}')
    client.close_connection()

    data = await client.get_last_state()

    # return f'Buzzer set to {value}'
    return jsonify(data)


if __name__ == '__main__':
    app.run()
