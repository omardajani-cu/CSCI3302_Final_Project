import config


def gripper():
    if (config.gripper_status == "open"):
        # Close gripper, note that this takes multiple time steps...
        config.robot_parts["gripper_left_finger_joint"].setPosition(0)
        config.robot_parts["gripper_right_finger_joint"].setPosition(0)
        if config.right_gripper_enc.getValue() <= 0.005:
            config.gripper_status = "closed"
    else:
        # Open gripper
        config.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        config.robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if config.left_gripper_enc.getValue() >= 0.044:
            config.gripper_status = "open"


def initManipulator():
	global MANIPULATOR_ARM_PARTS, DEFAULT_MANIPULATOR_ENCODING
	MANIPULATOR_ARM_PARTS = ["torso_lift_joint", "arm_1_joint", "arm_2_joint",  "arm_3_joint",  "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
	DEFAULT_MANIPULATOR_ENCODING = [0.15, 1.584,0.037,-3.262,-0.085,1.320,0,1.410]
	
def setMotors(motor_encodings):
	for i in range(len(motor_encodings)):
		config.robot_parts[MANIPULATOR_ARM_PARTS[i]].setPosition(motor_encodings[i])

def moveForward():
	shelf_forward_0 = [0, 1.662, -0.240, -1.575,  -0.320,  1.987, 0.0,  1.410]
	pass

# hardcode to attempt to pick up object on each level of the shelf
def searchShelves():
	shelf_encode_0 = [0, 1.662, -0.240, -1.575,  -0.320,  1.987, 0.0,  1.410]
	shelf_encode_1 = []
	shelf_encode_2 = []

	
