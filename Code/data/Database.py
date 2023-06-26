from owlready2 import *
from .Data import Data

DB_ABSOLUTE_PATH = os.path.dirname(os.path.abspath(__file__))
OWLFILE = "/4WD_Car_ontology_specific.owl/"
OWLIRI = "http://www.semanticweb.org/fenrir/ontologies/2023/5/4WD_car_specific_ontology/#"
SQLITE3FILE = "/4WD_car_db.sqlite3"


def init_database():
    """
    Create the .slite3 file with the ontology
    """
    default_world.set_backend(filename=DB_ABSOLUTE_PATH + SQLITE3FILE, exclusive=False)
    onto = get_ontology("file://" + DB_ABSOLUTE_PATH + OWLFILE).load()
    print_ontology(onto)
    default_world.save()
    return onto


def start_database():
    """
    Set the backend of the current default_world to the .sqlite3 file, if this file doesn't exist create it
    Return the ontology
    """
    if not os.path.exists(DB_ABSOLUTE_PATH+SQLITE3FILE):
        return init_database()
    else:
        if os.path.exists(DB_ABSOLUTE_PATH + SQLITE3FILE + "-journal"):
            print("The database has been closed without calling stop_database, some changes may has been lost")
            os.remove(DB_ABSOLUTE_PATH + SQLITE3FILE + "-journal")
        default_world.set_backend(filename=DB_ABSOLUTE_PATH + SQLITE3FILE, exclusive=False)
        return default_world.get_ontology(DB_ABSOLUTE_PATH+OWLFILE).load()


def stop_database():
    """
    Save the current state of the ontology in the data and close it, must be called before shutdown
    """
    default_world.save()
    default_world.close()


def print_ontology(onto: Ontology):
    print(f'{"Classes":-^30}')
    for classes in onto.classes():
        print(classes)
    print(f'{"Object properties":-^30}')
    for props in onto.object_properties():
        print(props)
    print(f'{"data properties":-^30}')
    for data in onto.data_properties():
        print(data)
    print(f'{"":->40}')


def add_car_data_to_db(data: Data, onto: Ontology):
    """
    Add the current data object (which represent the state of the car at a given moment) in the ontology
    If the car doesn't exist create it
    """
    existing_car = onto.search_one(is_a=onto.car, MAC_address=data.Car_MAC_address)
    if existing_car is None:
        existing_car = create_4WD_car(onto=onto, mac=data.Car_MAC_address, ip=data.Car_IP_address)
    add_measured_data(car=existing_car, onto=onto, data=data)


def add_measured_data(car, onto: Ontology, data: Data):
    """
    Call the auto_map function for each part and subAssembly of the car
    Also update the IP address of the car
    """
    car.IP_address = data.Car_IP_address
    for part in car.hasPart:
        auto_map_part(part, onto=onto, data=data)
    for subAssembly in car.hasSubAssembly:
        auto_map_sub_assembly(subAssembly, onto=onto, data=data)


def auto_map_sub_assembly(sub, onto: Ontology, data: Data):
    """
    Automatically add the right data in the right field of the ontology for each subAssembly
    """
    if isinstance(sub, onto.led_strip):
        rgb = data.leds_state
        for led in sub.hasPart:
            new_measure = onto.measure()
            new_measure.timestamp = data.timestamp
            index = led.index
            new_measure.red_intensity = (rgb[index][0])
            new_measure.green_intensity = (rgb[index][1])
            new_measure.blue_intensity = (rgb[index][2])
            led.hasMeasure.append(new_measure)

    elif isinstance(sub, onto.powertrain):
        motor_model = data.motor_model
        for motor in sub.hasPart:
            new_measure = onto.measure()
            new_measure.timestamp = data.timestamp
            position = motor.position
            new_measure.position = position
            if position == "FR":
                new_measure.speed = (motor_model[0])
            elif position == "FL":
                new_measure.speed = (motor_model[1])
            elif position == "RR":
                new_measure.speed = (motor_model[2])
            elif position == "RL":
                new_measure.speed = (motor_model[3])
            motor.hasMeasure.append(new_measure)


def auto_map_part(part, onto: Ontology, data: Data):
    """
    Automatically add the right data in the right field of the ontology for each part
    """
    new_measure = onto.measure()
    new_measure.timestamp = data.timestamp
    if isinstance(part, onto.battery):
        new_measure.voltage = data.battery_voltage
        new_measure.charge = data.battery_percent

    elif isinstance(part, onto.camera):
        new_measure.is_recording = data.camera_is_recording
        new_measure.framerate = data.camera_framerate
        new_measure.resolution_height = data.camera_resolution_height
        new_measure.resolution_width = data.camera_resolution_width

    elif isinstance(part, onto.CPU):
        new_measure.use_percent = data.CPU_use_percent
        new_measure.number_of_process = data.nb_process
        new_measure.samplin_rate = data.samplin_rate

    elif isinstance(part, onto.buzzer):
        new_measure.is_buzzing = data.buzzer_inUse

    elif isinstance(part, onto.ultrasonic):
        new_measure.ultrasonicOn = data.ultrasonic_inUse

    part.hasMeasure.append(new_measure)


def create_4WD_car(onto: Ontology, mac: str, ip: str):
    """
    Create a new car in the ontology
    Using the model of a 4WD Freenove Car
    """
    records = onto.records()
    battery = onto.battery()
    cpu = onto.CPU()
    camera = onto.camera()
    buzzer = onto.buzzer()
    ultrasonic = onto.ultrasonic()

    led1 = onto.led(index=0)
    led2 = onto.led(index=1)
    led3 = onto.led(index=2)
    led4 = onto.led(index=3)
    led5 = onto.led(index=4)
    led6 = onto.led(index=5)
    led7 = onto.led(index=6)
    led8 = onto.led(index=7)
    led_strip = onto.led_strip(hasPart=[led1, led2, led3, led4, led5, led6, led7, led8])

    motorFL = onto.motor(position="FR")
    motorFR = onto.motor(position="FL")
    motorRL = onto.motor(position="RR")
    motorRR = onto.motor(position="RL")
    powertrain = onto.powertrain(hasPart=[motorFL, motorFR, motorRL, motorRR])

    car = onto.car(MAC_address=mac, IP_address=ip, recorded=[records], hasSubAssembly=[powertrain, led_strip],
                   hasPart=[battery, cpu, camera, buzzer, ultrasonic])
    return car


############################################################################
# Ontology that we will be using built on top of an abstracte one

# ----------------Classes----------------
# 4WD_Car_ontology_specific.Assembly         ---> Abstract class that hasPart
# 4WD_Car_ontology_specific.Part             ---> Abstract class
# 4WD_Car_ontology_specific.measure
# 4WD_Car_ontology_specific.car
# 4WD_Car_ontology_specific.records
# 4WD_Car_ontology_specific.CPU
# 4WD_Car_ontology_specific.battery
# 4WD_Car_ontology_specific.buzzer
# 4WD_Car_ontology_specific.camera
# 4WD_Car_ontology_specific.ultrasonic
# 4WD_Car_ontology_specific.led_strip        ---> hasPart led(s)
# 4WD_Car_ontology_specific.powertrain       ---> hasPart motor(s)
# 4WD_Car_ontology_specific.led
# 4WD_Car_ontology_specific.motor
# ---------------------------------------
# -----------Object_properties-----------
# 4WD_Car_ontology_specific.hasSubAssembly    ---> Default relation
# 4WD_Car_ontology_specific.hasPart           ---> Default relation
# 4WD_Car_ontology_specific.hasMeasure
# 4WD_Car_ontology_specific.recorded
# ---------------------------------------
# ------------Data_properties------------
# 4WD_Car_ontology_specific.IP_address       ---\ Specific
# 4WD_Car_ontology_specific.MAC_address      ---/ to the car class
# 4WD_Car_ontology_specific.blue_intensity
# 4WD_Car_ontology_specific.charge
# 4WD_Car_ontology_specific.file_path
# 4WD_Car_ontology_specific.framerate
# 4WD_Car_ontology_specific.green_intensity
# 4WD_Car_ontology_specific.samplin_rate
# 4WD_Car_ontology_specific.is_buzzing
# 4WD_Car_ontology_specific.is_recording
# 4WD_Car_ontology_specific.number_of_process
# 4WD_Car_ontology_specific.red_intensity
# 4WD_Car_ontology_specific.resolution_height
# 4WD_Car_ontology_specific.resolution_width
# 4WD_Car_ontology_specific.speed
# 4WD_Car_ontology_specific.timestamp
# 4WD_Car_ontology_specific.ultrasonicOn
# 4WD_Car_ontology_specific.use_percent
# 4WD_Car_ontology_specific.voltage
# ---------------------------------------
