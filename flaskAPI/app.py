from flask import Flask, request, session, render_template, url_for, redirect

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

        return redirect(url_for('choices'))

    return render_template('index.html')


@app.route('/choices')
def choices():
    return render_template('choices.html')


@app.route('/toggleLed')
def toggle_led():
    client = Client(session.get('ip'), session.get('port'), session.get('port'))
    value = request.args.get('value')
    client.send_msg(f'led {value}')
    client.close_connection()

    return f'LED set to {value}'


@app.route('/toggleBuzzer')
def toggle_buzzer():
    client = Client(session.get('ip'), int(session.get('port')))
    value = request.args.get('value')
    client.send_msg(f'buzzer {value}')
    client.close_connection()

    return f'Buzzer set to {value}'


if __name__ == '__main__':
    app.run()
