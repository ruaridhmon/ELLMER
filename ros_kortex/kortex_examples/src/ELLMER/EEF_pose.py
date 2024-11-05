#!/usr/bin/env python3
import sys
import rospy

from kortex_driver.srv import *
from kortex_driver.msg import *
from base import ExampleMoveItTrajectories

import numpy as np
from geometry_msgs.msg import WrenchStamped

from kortex_driver.srv import *
from kortex_driver.msg import *

from sensor_msgs.msg import JointState
import optas
from urdf_parser_py.urdf import URDF
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

class CalculatePose:
    
    def __init__(self):
        self.robot = ExampleMoveItTrajectories()

        self.rotation_matrix_publisher = rospy.Publisher('rotation_matrix_topic', Float32MultiArray, queue_size=10)

        self._link_ee = rospy.get_param('~link_ee', 'link_ee')

        param_robot_description = '~/robot_description_kortex'
        if rospy.has_param(param_robot_description):
            self._robot_description = rospy.get_param(param_robot_description)
            self._urdf = URDF.from_parameter_server(param_robot_description)
        else:
            rospy.logerr("%s: Param %s is unavailable!" % (self._name, param_robot_description))
            rospy.signal_shutdown('Incorrect parameter name.')


        self.robot= optas.RobotModel(
                urdf_string=self._robot_description,
                time_derivs=[0],
                param_joints=['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7'],
                name='kortex_robot'
            )
        self.quaternion_fnc = self.robot.get_global_link_quaternion_function(link=self._link_ee)
        self.quaternion = np.zeros(4)
        self.rotation_matrix_fnc = self.robot.get_global_link_rotation_function(link=self._link_ee)
        self.rotation_matrix = np.zeros((3,3))
        self.rpy_fnc = self.robot.get_global_link_rpy_function(link=self._link_ee)
        self.rpy = np.zeros(3)

        self.global_force_publisher = rospy.Publisher('global_force', WrenchStamped, queue_size=10)
        rospy.Subscriber("/netft/netft_data", WrenchStamped, self.netft_data_callback)
        self.rotation_matrix = None
 
    def EEF_pose_callback(self, data):
        self.quaternion = self.quaternion_fnc(np.array(data.position)[0:7])
        self.rotation_matrix = self.rotation_matrix_fnc(np.array(data.position)[0:7])

        # Convert the CasADi DM object to a NumPy array and then flatten it
        rotation_matrix_np = np.array(self.rotation_matrix)  # Convert to NumPy array
        rotation_matrix_flat = rotation_matrix_np.flatten().tolist()  # Flatten and convert to list

        # Prepare the Float32MultiArray message
        rotation_matrix_msg = Float32MultiArray()
        rotation_matrix_msg.layout.dim.append(MultiArrayDimension())
        rotation_matrix_msg.layout.dim[0].size = rotation_matrix_np.size
        rotation_matrix_msg.layout.dim[0].stride = rotation_matrix_np.shape[0] * rotation_matrix_np.shape[1]
        rotation_matrix_msg.layout.dim[0].label = 'rotation_matrix'
        rotation_matrix_msg.data = rotation_matrix_flat

        # Publish the message
        self.rotation_matrix_publisher.publish(rotation_matrix_msg)

    def netft_data_callback(self, data):
        if self.rotation_matrix is not None:
            force = data.wrench.force
            force_array = np.array([force.x, force.y, force.z])
            force_global = np.dot(self.rotation_matrix, force_array)
            torque = data.wrench.torque
            torque_array = np.array([torque.x, torque.y, torque.z])
            torque_global = np.dot(self.rotation_matrix, torque_array)

            # Publish the global force
            wrench_global = WrenchStamped()
            wrench_global.header.stamp = rospy.Time.now() 
            wrench_global.wrench.force.x = force_global[0]
            wrench_global.wrench.force.y = force_global[1]
            wrench_global.wrench.force.z = force_global[2]
            wrench_global.wrench.torque.x = torque_global[0]
            wrench_global.wrench.torque.y = torque_global[1]
            wrench_global.wrench.torque.z = torque_global[2]
            self.global_force_publisher.publish(wrench_global)



interface = CalculatePose()
rospy.Subscriber("/my_gen3/base_feedback/joint_state", JointState, interface.EEF_pose_callback)

rospy.spin()