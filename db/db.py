from owlready2 import *

if __name__ == "__main__":

    onto = get_ontology("file:///home/fenrir/INFO4/stage/4WD_Car_onto.owl").load()
    for truc in onto.classes():
        print(truc)
    #default_world.set_backend(filename= "4WD_smart_car_db.sqlite3")
    #default_world.save()
    # Spécifier le fichier de base de données SQLite
    # onto_path.append("4WD_smart_car_db.sqlite3")


    