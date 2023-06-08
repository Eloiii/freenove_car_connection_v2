# [freenove-car-connection-v2](https://github.com/Eloiii/freenove_car_connection_v2)

An environment to control and manage a [Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi).

## Structure

### `tcp_connections`

A folder containing a client and a server over TCP.
- server is the car waiting for commands
- client is another device sending commands to the car

Three sockets can be opened :
- one for sending commands to the car (e.g. rotate servos, light up LEDs...)
- one for the camera stream
- one for retrieving car information (e.g. CPU usage, battery percentage...)

### `app.py`

A flask server used to control the car via HTTP.

Each HTTP endpoint opens a socket in order to perform action on the car.

`./tcp_connection/car_utilities` contains files from [original repo](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/tree/master/Code)

## Usage

### Server

Launch `sudo python server.py` on the car

### Client

- Manual launch : `python client.py <car_ip>` on another device connected to the same network as the car.

- Run `./venv/bin/flask --app app run` to start the Flask app.

#### Endpoints

To set up the TCP connection between the client and the server (the car) the car IP address is required in
**at least the first endpoint call as a GET parameter** (except `/`, e.g. `localhost:5000/start_recording?ip=138.250.156.7`).

Once the IP is filled in once, it is not mandatory when calling other endpoints after that.

- `/`
- `/setLED` : 
  - parameter `value` : `LEDINDEX_REDVALUE_GREENVALUE_BLUEVALUE`
  - e.g. `value=0x01_255_255_255` will turn the LED 1 white
- `/setMotors` :
  - parameter `value` : `LEFTUPPERWHEEL_LEFTLOWERWHEEL_RIGHUPPERWHEEL_RIGHTLOWERWHEEL_`
  - e.g. `value=2000_2000_2000_2000` will make the car go forward
- `/setServo` :
  - parameter `value` : `SERVOID_DEGREES`
  - e.g. `value=0_90` will turn the servo 0 to 90 degrees
- `/toggleBuzzer` :
  - parameter `value` : `ACTIVATIONVALUE`
  - e.g. `value=0` will turn on the buzzer
- `/get_video` : live-streaming of the car camera (⚠️ takes a lot of resources (RAM and CPU%))
  - parameter `framerate` : number of images per second recorded
  - e.g. `framerate=20` will capture a real-time video in 20 frames per second
- `/start_recording` : start a recording, saving images to local device
  - parameter `framerate` : number of images per second recorded
  - e.g. `framerate=20` will capture a video in 20 frames per second
- `/stop_recording` : stop the current recording

