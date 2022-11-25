import config     

# returns position of yellow detected object
def detect_object():
    objs = config.camera.getRecognitionObjects()
    for i, obj in enumerate(objs):
        # if obj.getModel().decode("utf-8") == 'cereal box':
        #     print("YAY")
        color = obj.getColors()
        if color[0] == 1 and color[1] == 1 and color[2] == 0:
            # print("Yellow detected")
            # print("Pos = " + str(objs[i].getPosition()))
            return objs[i].getPosition()

    return []

    # [<controller.CameraRecognitionObject; proxy of <Swig Object of type 'webots::CameraRecognitionObject *' at 0x00000296BA2BC690> >]


