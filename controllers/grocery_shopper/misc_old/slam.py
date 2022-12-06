import config     

ID_POS_MAP = {
    4427: (-2.35,0.3,0.53),
    
}


# returns position of yellow detected object
def detect_object():

    current_ids = []
    current_positions = []

    objs = config.camera.getRecognitionObjects()
    for i, obj in enumerate(objs):
        current_ids.append(obj.getId())
        current_positions.append(obj.getPosition())

    print("Current ids: = " + str(current_ids))
    print("Current positions = " + str(current_positions))



