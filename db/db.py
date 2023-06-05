from owlready2 import *
#from tcp_connections.car_utilities.DataCollection import *


def print_ontology(onto:Ontology):
    print("----------------Classes----------------")
    for classes in onto.classes():
        print(classes)
    print("---------------------------------------")
    print("-----------Object_properties-----------")
    for props in onto.object_properties():
        print(props)
    print("---------------------------------------")
    print("------------Data_properties------------")
    for data in onto.data_properties():
        print(data)
    print("---------------------------------------")
    
#def add_car_data_to_db(data:Data, onto:Ontology):
#    existing_car = onto.search_one(is_a= onto.Car, MAC_Address=data.Car_MAC_address)
#    if(existing_car != None):
#        add_data_entry(existing_car, data=data)
#    else:
#        add_data_entry(create_car(onto=onto, MAC=data.Car_MAC_address), data)

def add_measured_data(car,onto:Ontology):#, data:Data):
   pass

def create_4WD_car(onto:Ontology, MAC:str, IP:str):
    records = onto.records()
    battery = onto.battery()
    cpu = onto.CPU()
    camera = onto.camera()
    buzzer = onto.buzzer()
    ultrasonic = onto.ultrasonic()

    led1 = onto.led(index=1)
    led2 = onto.led(index=2)
    led3 = onto.led(index=3)
    led4 = onto.led(index=4)
    led5 = onto.led(index=5)
    led6 = onto.led(index=6)
    led7 = onto.led(index=7)
    led8 = onto.led(index=8)
    led_strip = onto.led_strip(hasPart=[led1,led2,led3,led4,led5,led6,led7,led8])

    motorFL = onto.motor(position="FL")
    motorFR = onto.motor(position="FR")
    motorRL = onto.motor(position="RR")
    motorRR = onto.motor(position="RL")
    powertrain = onto.powertrain(hasPart=[motorFL,motorFR,motorRL,motorRR])
    
    car = onto.Car(MAC_address=MAC, IP_address=IP, recorded=[records], hasSubAssembly=[powertrain,led_strip], hasPart=[battery,cpu,camera,buzzer,ultrasonic])
    return car
    

if __name__ == "__main__":
    default_world.set_backend(filename= "4WD_smart_car_db.sqlite3")
    onto = default_world.get_ontology("file:///home/fenrir/INFO4/stage/4WD_Car_ontology_specific.owl").load()
    print_ontology(onto=onto)
    #create_car(onto=onto, MAC="TEST:CAR:TO:REMOVE")
    #for individual in list(onto.individuals()): 
    #    print(individual)
    #    for prop in individual.get_properties():
    #        for value in prop[individual]:
    #            print("%s == %s" % (prop.python_name, value))
    default_world.save()

############################################################################
#Ontology that we will be using built on top of an abstracte one

#----------------Classes----------------
#4WD_Car_ontology_specific.Assembly         ---> Abstract class that hasPart
#4WD_Car_ontology_specific.Part             ---> Abstract class
#4WD_Car_ontology_specific.measure
#4WD_Car_ontology_specific.car
#4WD_Car_ontology_specific.records
#4WD_Car_ontology_specific.CPU
#4WD_Car_ontology_specific.battery
#4WD_Car_ontology_specific.buzzer
#4WD_Car_ontology_specific.camera
#4WD_Car_ontology_specific.ultrasonic
#4WD_Car_ontology_specific.led_strip        ---> hasPart led(s)
#4WD_Car_ontology_specific.powertrain       ---> hasPart motor(s)
#4WD_Car_ontology_specific.led
#4WD_Car_ontology_specific.motor
#---------------------------------------
#-----------Object_properties-----------
#4WD_Car_ontology_specific.hasSubAssembly    ---> Default relation
#4WD_Car_ontology_specific.hasPart           ---> Default relation
#4WD_Car_ontology_specific.hasMeasure
#4WD_Car_ontology_specific.recorded
#---------------------------------------
#------------Data_properties------------
#4WD_Car_ontology_specific.IP_address       ---\ Specific
#4WD_Car_ontology_specific.MAC_address      ---/ to the car class
#4WD_Car_ontology_specific.blue_intensity
#4WD_Car_ontology_specific.charge
#4WD_Car_ontology_specific.file_path
#4WD_Car_ontology_specific.framerate
#4WD_Car_ontology_specific.green_intensity
#4WD_Car_ontology_specific.samplin_rate
#4WD_Car_ontology_specific.is_buzzing
#4WD_Car_ontology_specific.is_recording
#4WD_Car_ontology_specific.number_of_process
#4WD_Car_ontology_specific.red_instensity
#4WD_Car_ontology_specific.resolution_height
#4WD_Car_ontology_specific.resolution_width
#4WD_Car_ontology_specific.speed
#4WD_Car_ontology_specific.timestamp
#4WD_Car_ontology_specific.ultrasonicOn
#4WD_Car_ontology_specific.use_percent
#4WD_Car_ontology_specific.voltage
#---------------------------------------


