import json
from datetime import datetime

class Data:

    def __init__(self):
        self.Car_MAC_address = None
        self.Car_IP_address = None
        self.samplin_rate = None
        self.timestamp = None
        self.battery_percent = None
        self.battery_voltage = None
        self.camera_is_recording = None
        self.camera_resolution_height = None
        self.camera_resolution_width = None
        self.camera_framerate = None
        self.CPU_use_percent = None
        self.nb_process = None
        self.motor_model = None #A list of the use of each wheel FR|FL|BR|BL
        self.leds_state= None # A list of RGB code for each Leds
        self.ultrasonic_inUse = None
        self.buzzer_inUse = None

def setData(data:Data,MAC:str=None,IP:str=None,sampl_rate:int=None,battery_voltage:float=None,battery_percent:float=None,isRecording:bool=None,height=None,width=None,FPS=None,CPU:float=None,nb_process:int=None,motor_model=None,leds=None, ultrasonic:bool=None, buzzer:bool=None):
    '''
    Used by the server to update the current state of the car in the Data object
    '''
    data.timestamp = datetime.timestamp(datetime.now())
    if MAC!=None:
        data.Car_MAC_address = MAC
    if IP!=None:
        data.Car_IP_address = IP
    if sampl_rate!=None:
        data.samplin_rate = sampl_rate
    if battery_voltage!=None:
        data.battery_voltage = battery_voltage
    if battery_percent!=None:
        data.battery_percent = battery_percent
    if isRecording!=None:
        data.camera_is_recording = isRecording
    if height!=None:
        data.camera_resolution_height = height
    if width!=None:
        data.camera_resolution_width = width
    if FPS!=None:
        data.camera_framerate = FPS
    if CPU!=None:
        data.CPU_use_percent = CPU
    if nb_process!=None:
        data.nb_process = nb_process
    if motor_model!=None:
        data.motor_model = motor_model
    if leds!=None:
        data.leds_state = leds
    if ultrasonic!=None:
        data.ultrasonic_inUse = ultrasonic
    if buzzer!=None:
        data.buzzer_inUse = buzzer

def printData(data:Data):
    print("____________________________________________________________________________________________________\n"
      f'Car MAC address: {data.Car_MAC_address}\n'
      f'Car IP Address: {str(data.Car_IP_address)}\n'
      f'Sampling Rate: {data.samplin_rate}s\n'
      f'Timestamp: {datetime.fromtimestamp(data.timestamp)}\n'
      f'Battery: {data.battery_voltage}V {data.battery_percent}%\n'
      f'CPU: {data.CPU_use_percent}% | Number of process: {data.nb_process}\n'
      f'Camera: Is recording: {data.camera_is_recording} | '
      f'Resolution: {data.camera_resolution_width}x{data.camera_resolution_height} {data.camera_framerate}FPS\n'
      f'Wheels:\nFR: {data.motor_model[0]} | FL: {data.motor_model[1]} | RR: {data.motor_model[2]} | RL: {data.motor_model[3]}\n'
      f'LedsBrightness: {data.leds_state}\n'
      f'Ultrasonic: {data.ultrasonic_inUse}\n'
      f'Buzzer: {data.buzzer_inUse}'
      "\n____________________________________________________________________________________________________")
