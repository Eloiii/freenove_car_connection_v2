from owlready2 import *
from rdflib import Graph

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
    


if __name__ == "__main__":

    default_world.set_backend(filename= "4WD_smart_car_db.sqlite3")
    #onto = get_ontology("file://4WD_Smart_Car_onto.owl").load()

    #test = onto.Car("Test")
    #test.IP_Address.append("127.0.0.1")
    #test.MAC_Address.append("00:1A:2B:3C:4D:ZZ")

    for truc in default_world.individuals():
        print(truc)
        print(truc.IP_Address)
        print(truc.MAC_Address)
    
    #default_world.save()


############################################################################
#4WD_SMART_CAR_ONTOLOGY
#----------------Classes----------------
#Car
#equipment
#records
#CPU
#buzzer
#camera
#led1
#led2
#led3
#led4
#led5
#led6
#led7
#led8
#ultrasonic
#wheelFL
#wheelFR
#wheelRL
#wheelRR
#battery
#---------------------------------------
#-----------Object_properties-----------
#hasPart
#isPartOf
#record
#use
#---------------------------------------
#------------Data_properties------------
#IP_Address
#MAC_Address
#blue_intensity
#buzzing
#charge
#file_path
#framerate
#green_intensity
#number_of_process
#recording
#red_intensity
#resolution_height
#resolution_width
#speed
#timestamp
#ultrasonicOn
#use_percent
#voltage
#---------------------------------------
