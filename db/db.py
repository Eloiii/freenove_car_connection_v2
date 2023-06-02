from owlready2 import *
from tcp_connections.car_utilities.DataCollection import *


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
    
def add_car_data_to_db(data:Data, onto:Ontology):
    #onto.search_one(is_a= onto.Car, MAC_Address=data.)
    pass

if __name__ == "__main__":
    default_world.set_backend(filename= "4WD_smart_car_db.sqlite3")
    onto = default_world.get_ontology("http://www.semanticweb.org/fenrir/ontologies/2023/5/4WD-smart-car-ontology#").load()
    #car = onto.search_one(is_a = onto.Car, IP_Address="127.0.0.1")
    for individual in list(onto.individuals()): 
        print(individual)
        for prop in individual.get_properties():
            for value in prop[individual]:
                print("%s == %s" % (prop.python_name, value))

    
    #default_world.save()

#Du coup faire set_backend ça suffit ?? mettre des objets via ça c'est good ?
#Est-ce que je dois modifier l'IRI ? Est-ce que je peu ?
#Comment faire des requetes SPARQL ?

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
