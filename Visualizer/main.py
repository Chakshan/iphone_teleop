import time
from g1_29_armIK import G1_29_ArmIK
from iphone_pos_tracker import iPhonePosTracker
from pynput import keyboard
import numpy as np

hand_poses = {
    "open-palm": [0, 0, 0, 0, 0, 0],
    "closed-fist": [1, 1, 1.25, 1.25, 1.25, 1.25]
}

key_to_pose = {
    '1': "open-palm",
    '2': "closed-fist"
}

last_key = '1'

def on_press(key):
    global last_key
    try:
        if hasattr(key, 'char'):
            if (key.char in key_to_pose.keys()):
                last_key = key.char
    except AttributeError:
        print(f'Special key pressed: {key}')

if __name__ == "__main__":
    arm_ik = G1_29_ArmIK(Unit_Test=True, Visualization=True)
    phoneTracker = iPhonePosTracker()
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    user_input = input("Please enter the start signal (enter 's' to start the subsequent program):\n")
    if user_input.lower() == 's':

        
        step = 0

        while True:
            phone_tf = phoneTracker.get_tf()

            # Transformations for left and right wrists
            L_tf_target = arm_ik.L_tf_init                  # default pose
            R_tf_target = arm_ik.R_tf_init @ phone_tf       # phone pose compounded with init position

            left_hand_q  = [0, 0, 0, 0, 0, 0]  
            right_hand_q = hand_poses[key_to_pose[last_key]]

            arm_ik.move_arms_and_hands(L_tf_target, R_tf_target, left_hand_q, right_hand_q)

            
            # Translation of poses
            phone_transl = phone_tf[0:3, 3] / phone_tf[3, 3]
            L_tf_transl  = L_tf_target[0:3, 3] / L_tf_target[3, 3]
            R_tf_transl  = R_tf_target[0:3, 3] / R_tf_target[3, 3]

            # Print stats after some number of steps
            if step % 3 == 0:
                print(f"iPhone  : x={phone_transl[0]:>3.4f}, y={phone_transl[1]:>3.4f}, z={phone_transl[2]:>3.4f}")
                print(f"Left_tf : x={L_tf_transl[0]:>3.4f}, y={L_tf_transl[1]:>3.4f}, z={L_tf_transl[2]:>3.4f}")
                print(f"Right_tf: x={R_tf_transl[0]:>3.4f}, y={R_tf_transl[1]:>3.4f}, z={R_tf_transl[2]:>3.4f}")
                print()
                print(f"Left hand pose: open-palm")
                print(f"Right hand pose: {key_to_pose[last_key]:15}")
                print("\033[F" * 7)
                step = 0
            
            step += 1
            time.sleep(0.1)