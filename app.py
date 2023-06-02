from flask import Flask, request, render_template, jsonify, Response

from tcp_connections.client import *

app = Flask(__name__)
app.secret_key = 'SECRET_KEY'


@app.route('/')
def index():
    return render_template('choices.html')


@app.errorhandler(500)
def internal_server_error(error):
    return render_template('internal_server_error.html', error=error), 500


async def send_msg_and_receive_state(command):
    client = Client()
    client.send_msg(command)

    data = await client.get_last_state()
    return data


@app.before_request
def setup_client_connection():
    ip = request.args.get('ip')
    port = request.args.get('port')
    client = Client()
    if not client.initialised and ip is not None:
        client.setup(ip, int(port if port is not None else '8787'))


@app.route('/setLED')
async def set_led():
    value = request.args.get('value')

    try:
        state = await send_msg_and_receive_state(f'led {value}')
    except Exception as e:
        return e.__dict__

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


def video():
    client = Client()
    client.connect_to_video_server()
    while True:
        if client.imgbytes is not None:
            yield b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + client.imgbytes + b'\r\n'


@app.route('/get_video')
def get_video():
    return Response(video(), mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/start_recording')
def start_recording():
    client = Client()
    framerate = request.args.get('framerate')
    client.connect_to_video_server(framerate)

    return 'Recording...'


@app.route('/stop_recording')
def stop_recording():
    client = Client()
    client.close_video_connection()

    return 'Recording stopped'


if __name__ == '__main__':
    app.run()
