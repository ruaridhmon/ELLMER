#!/usr/bin/env python3
import rospy

from kortex_driver.srv import *
from kortex_driver.msg import *
from std_msgs.msg import String

from base import ExampleMoveItTrajectories
import numpy as np

from matplotlib.dates import date2num
from kortex_driver.srv import *
from kortex_driver.msg import *
from geometry_msgs.msg import WrenchStamped

import rospy
import requests
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)
from action_execution import *
from std_msgs.msg import Float32MultiArray
from utilities_affordance import *

class Brain:
    
    def __init__(self):
        self.robot = ExampleMoveItTrajectories()
        self.eef_pose = None
        self.objects_positions = None
        self.object_labels = ["red cup"]

        self.current_pose = None

        self.class_publisher = rospy.Publisher('/update_classes', String, queue_size=10)

        history_size = 40
        self.history_size = history_size
        self.force_history_x = np.zeros(history_size)
        self.force_history_y = np.zeros(history_size)
        self.force_history_z = np.zeros(history_size)

        self.rotational_matrix = None
        self.objects = None
        self.avg_force_y = None


    def update_classes(self, updated_classes=["red cup"]):
        self.object_labels = updated_classes 
        classes_str = ", ".join(updated_classes)
        rospy.set_param('/classes', updated_classes)
        rospy.loginfo(classes_str)
        self.class_publisher.publish(classes_str)


    def rotation_matrix_callback(self, msg):
        flat_matrix = msg.data
        dim = int(np.sqrt(len(flat_matrix)))
        self.rotation_matrix = np.array(flat_matrix).reshape(dim, dim)

    def target_coordinates_callback(self, data):
        coordinates_list = []
        for coord in data.targetcoordinates:
            coordinates_list.append([coord.x-0.1, coord.y-0.17, coord.z+0.02])
        
        target_positions = np.array(coordinates_list)
        target_positions_labeled = {label: position for label, position in zip(self.object_labels, target_positions)}
        self.objects = target_positions_labeled


    def EEF_pose_callback(self, data):

        self.eef_pose = {
            "x": data.base.commanded_tool_pose_x,
            "y": data.base.commanded_tool_pose_y,
            "z": data.base.commanded_tool_pose_z,
            "theta_x": data.base.commanded_tool_pose_theta_x,
            "theta_y": data.base.commanded_tool_pose_theta_y,
            "theta_z": data.base.commanded_tool_pose_theta_z
        }

        self.current_pose = [self.eef_pose["x"], self.eef_pose["y"], self.eef_pose["z"], \
                        self.eef_pose["theta_x"], self.eef_pose["theta_y"], self.eef_pose["theta_z"]]
        self.current_position = self.current_pose[:3]

    

    def global_force_callback(self, data_global):
        # Shift history and add the newest data point at the end for each component

        self.wrench_global = data_global.wrench

        self.force_history_x = np.roll(self.force_history_x, -1)
        self.force_history_y = np.roll(self.force_history_y, -1)
        self.force_history_z = np.roll(self.force_history_z, -1)

        self.force_history_x[-1] = data_global.wrench.force.x
        self.force_history_y[-1] = data_global.wrench.force.y
        self.force_history_z[-1] = data_global.wrench.force.z

        self.avg_force_x = np.mean(self.force_history_x)
        self.avg_force_y = np.mean(self.force_history_y)
        self.avg_force_z = np.mean(self.force_history_z)

        self.force_up = self.avg_force_y


    def fetch_action(self):
        """Fetches an action (code snippet) from the Flask server and returns it."""
        response = requests.get('https://kinovaapi.com/get_action', verify=False)
        if response.status_code == 200:
            action_code = response.json()
            return action_code
        return None

    def action_listener(self):
        """Executes commands based on the fetched action code in a safe, controlled environment."""
        rate = rospy.Rate(2)
        print('listening')
        while not rospy.is_shutdown():
            action_code = self.fetch_action()
            if action_code:
                action = action_code.get('action')
                print(f'action {action}')
                if action['type'] == 'run_code':
                    code_str = action['payload'].get('code') 
                    print(code_str)
                    run_with_timeout(self, code_str)

            rate.sleep()

    def run_affordance(self):
        target_pose =[0.2, 0, 0.1, 170, 0, 100]
        target_pose2 = target_pose
        target_pose1 =[target_pose[0], target_pose[1], target_pose[2]+0.1, target_pose[3], target_pose[4], target_pose[5]]
        # 0 is close, 0.1 is open
        start_eef_position=0.055
        self.robot.example_send_gripper_goto(pos=start_eef_position)    
        # this function goes to one pose. May want to specify two points
        # if the approach angle is important
        go_to_pose_rotate_all_directions(self, target_pose1)
        go_to_pose_rotate_all_directions(self, target_pose2)
        self.robot.example_send_gripper_close(speed=0.02)

    def run_affordances2(self):
        self.robot.example_send_gripper_close(speed=0.02)
        go_to_pose_rotate_all_directions_dynamic(self, object_name="hand")        



    

interface = Brain()

# interface.robot.example_send_gripper_open()
# interface.robot.reach_named_position("home")
# interface.robot.reach_named_position("home")
# rospy.sleep(0.5)
# interface.robot.example_send_gripper_close(force=1)

rospy.Subscriber("/my_gen3/base_feedback", BaseCyclic_Feedback, interface.EEF_pose_callback)
rospy.Subscriber('/my_gen3/global_force', WrenchStamped, interface.global_force_callback)
rospy.Subscriber('/target_coordinates', TargetCoordinatesArray, interface.target_coordinates_callback)
rospy.Subscriber('/my_gen3/rotation_matrix_topic', Float32MultiArray, interface.rotation_matrix_callback)


while interface.avg_force_y is None:
    rospy.sleep(0.1)

while interface.objects is None:
    rospy.sleep(0.1)

interface.run_affordances2()

rospy.spin()