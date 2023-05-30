# freenove-car-connection-v2

An environment to control and manage a [Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi).

### Structure

#### `tcp_connections`

A folder containing a client and a server over TCP.
- server is the car waiting for commands
- client is another device sending commands to the car

Three sockets can be opened :
- one for sending commands to the car (e.g. rotate servos, light up LEDs...)
- one for the camera stream
- one for retrieving car information (e.g. CPU usage, battery percentage...)

#### `app.py`

A flask server used to control the car via HTTP.

Each HTTP endpoint opens a socket in order to perform action on the car.

`./tcp_connection/car_utilities` contains files from [original repo](https://github.com/Freenove/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/tree/master/Code)
