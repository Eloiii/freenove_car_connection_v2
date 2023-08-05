# [freenove-car-connection-v2](https://github.com/Eloiii/freenove_car_connection_v2)

An environment to control and manage a [Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi).

## Structure

The server is the car and a client is a remote host.

### Folder 'Code'

File `client.py` handles all TCP connections with the car. (used by Flask server)
Three sockets can be opened :
- one for sending commands to the car (e.g. rotate servos, light up LEDs...)
- one for the camera stream
- one for retrieving car information (e.g. CPU usage, battery percentage...)

Folder 'car_utilities' contains low-level code to interact with the car Raspberry Pi. (from [original repo](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/tree/master/Code))

### Flask application

The file `app.py` is the Flask application entry point.
A Flask server used to control the car via HTTP.
The Flask application handles the client side by creating sockets. Endpoints are used to control and manage the car. Multiple clients are allowed.

### Server

The file `server.py` is the server to be launched on the car. Handles incoming client connections and requests.

## Usage

### Server

Launch `sudo python3 server.py` on the car

### Client

- Run `./venv/bin/flask --app app run` to start the Flask app.

### Endpoints

First, initialize the connection by sending a GET request to `/connect/{car_pi}`.  
⚠️Every endpoint must have the `ci=` (client index) argument which indicates to which car to send the commands.⚠️

| Name | Required | Type    | Description   |
|------|----------|---------|---------------|
| `ci` | required | integer | The car index |

#### GET /connect/{car_pi}

set up the connection between the client and a car

#### GET /setLED

Lights an LED according to a given color.

##### Parameters

| Name    | Required | Type   | Description                                                                                                                                                          |
|---------|----------|--------|----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `value` | required | string | The LED index follow by three integers for RGB color.<br><br>`LEDINDEX_REDVALUE_GREENVALUE_BLUEVALUE`<br><br>e.g. `value=0x01_255_255_255` will turn the LED 1 white |

#### GET /setMotors

Change the speed of the wheels.

##### Parameters

| Name    | Required | Type   | Description                                                                                                                                                               |
|---------|----------|--------|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `value` | required | string | The speed for the four wheels.<br><br>`LEFTUPPERWHEEL_LEFTLOWERWHEEL_RIGHUPPERWHEEL_RIGHTLOWERWHEEL`<br><br>e.g. `value=2000_2000_2000_2000` will make the car go forward |

#### GET /setServo

Change the orientation of the servos.

##### Parameters

| Name    | Required | Type   | Description                                                                                                                             |
|---------|----------|--------|-----------------------------------------------------------------------------------------------------------------------------------------|
| `value` | required | string | The servo ID followed by the degree of the angle.<br><br>`SERVOID_DEGREES`<br><br>e.g. `value=0_90` will turn the servo 0 to 90 degrees |

#### GET /toggleBuzzer

Activate or deactivate the buzzer.

##### Parameters

| Name    | Required | Type    | Description                                                                                   |
|---------|----------|---------|-----------------------------------------------------------------------------------------------|
| `value` | required | integer | The activation value .<br><br>`ACTIVATIONVALUE`<br><br>e.g. `value=0` will turn on the buzzer |

#### GET /start_recording

Start a video recording.

##### Parameters

| Name        | Required | Type    | Description                               |
|-------------|----------|---------|-------------------------------------------|
| `framerate` | required | integer | The number of frames per second captured. |
| `height`    | required | integer | The height in pixels of the video.        |
| `width`     | required | integer | The width in pixels of the video.         |

#### GET /stop_recording

Stop the current video recording.

#### GET /data_collection_on

Activate the data collection.

#### GET /data_collection_off

Deactivate the data collection.
