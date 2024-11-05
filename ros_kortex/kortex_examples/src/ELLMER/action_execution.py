import rospy
import threading
from utilities_affordance import *
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

def execute_code(self, code):
    restricted_globals = {
        '__builtins__': {'print': print, 'range': range, 'float': float, 'time': time},
        'abs': abs,
        'rospy': rospy,
        'self': self,
        'calculate_unit_vector': calculate_unit_vector,
        'calculate_distance': calculate_distance,
        'go_to_point_velocity': go_to_point_velocity,
        'R': R,
        'numpy': np,
        'go_to_pose': go_to_pose,
        'len': len,
        'scoop': scoop,
        'shake': shake,
        'list': list,
        'go_to_pose_velocity': go_to_pose_velocity,
        'calculate_angular_displacement': calculate_angular_displacement,
        'get_object_position_with_wait': get_object_position_with_wait,
        'track_object': track_object,
        'go_to_position': go_to_position,
        'update_classes_function': update_classes_function,
        'empty_in_cup': empty_in_cup,
        'pour': pour,
        'go_to_spoon_from_above': go_to_spoon_from_above,
        'return_object_position': return_object_position,
        'go_through_trajectory': go_through_trajectory,
        'get_spoon': get_spoon,
        'return_spoon': return_spoon,
        'go_to_point_in_one_axis': go_to_point_in_one_axis,
        'get_kettle': get_kettle,
        'drawing': drawing,
        'get_mug_in_drawer': get_mug_in_drawer,
        'open_door_handle': open_door_handle,
        'close_door': close_door,
        'put_down_object': put_down_object,
        'track_and_pour': track_and_pour,
        'go_to_orientation': go_to_orientation,
        'give_item': give_item,
        'take_item': take_item,
        'go_to_coffee': go_to_coffee,
        'return_kettle': return_kettle,
        'go_to_pose_rotate_all_directions': go_to_pose_rotate_all_directions,
        'affordance_eef_to_object': affordance_eef_to_object,
    }

    # Debugging: Print the code to be executed
    print(f"Executing code: {code}")

    try:
        if isinstance(code, str):  # Ensure the code is a string
            exec(code, restricted_globals)
        else:
            raise TypeError(f"Code must be a string, got {type(code)}")
    except Exception as e:
        rospy.logerr(f"Error executing code: {e}")

def run_with_timeout(self, code, timeout=400):
    t = threading.Thread(target=execute_code, args=(self,code))
    t.start()
    t.join(timeout)
    if t.is_alive():
        print("Terminating due to timeout")