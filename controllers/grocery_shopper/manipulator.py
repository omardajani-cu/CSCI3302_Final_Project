import config

def gripper(): 
    if(config.gripper_status=="open"):
        # Close gripper, note that this takes multiple time steps...
        config.robot_parts["gripper_left_finger_joint"].setPosition(0)
        config.robot_parts["gripper_right_finger_joint"].setPosition(0)
        if config.right_gripper_enc.getValue()<=0.005:
            config.gripper_status="closed"
    else:
        # Open gripper
        config.robot_parts["gripper_left_finger_joint"].setPosition(0.045)
        config.robot_parts["gripper_right_finger_joint"].setPosition(0.045)
        if config.left_gripper_enc.getValue()>=0.044:
            config.gripper_status="open" 