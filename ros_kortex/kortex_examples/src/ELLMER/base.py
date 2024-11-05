#! /usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import math
from moveit_commander import PlanningSceneInterface
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from kortex_driver.srv import *
from kortex_driver.msg import *

#
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperAction
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from copy import deepcopy


class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("/my_gen3/robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      self.twist_publisher = rospy.Publisher('/my_gen3/in/cartesian_velocity', TwistCommand, queue_size=20)
      self.robot_name = rospy.get_param('~robot_name', "my_gen3")

      self.action_name = rospy.get_param('~action_name', '/command_robotiq_action')
      self.robotiq_client = actionlib.SimpleActionClient(self.action_name, CommandRobotiqGripperAction)
      self.robotiq_client.wait_for_server()

      self.eef_pose_subscriber = rospy.Subscriber("/my_gen3/base_feedback", BaseCyclic_Feedback, self.EEF_pose_callback)
      self.timer = rospy.Timer(rospy.Duration(0.1), self.check_bounds_and_publish)      
      self.current_velocity = None
      self.current_position  = None

      self.planning_scene_interface = PlanningSceneInterface()

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

    print(self.is_init_success)

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


  def reach_named_position(self, target):
    arm_group = self.arm_group
    rospy.loginfo("Going to named target " + target)
    arm_group.set_named_target(target)
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    return arm_group.execute(trajectory_message, wait=True)


  def clamp_velocity(self, value, min_value, max_value):
      return max(min(value, max_value), min_value)

  def spatial_bounds(self, position, velocity, time_step=0.1, buffer_margin=0.01):
      spatial_bounds = [(0, 1.2), (-0.4, 0.4), (0.055, 0.7)] # 0.07
      
      new_velocity = deepcopy(velocity)

      # Predict the next position
      next_position = [position[i] + velocity[i] * time_step for i in range(3)]

      for i in range(3):
          if next_position[i] < spatial_bounds[i][0] + buffer_margin:
              new_velocity[i] = max(0, new_velocity[i])
          elif next_position[i] > spatial_bounds[i][1] - buffer_margin:
              new_velocity[i] = min(0, new_velocity[i])

      return new_velocity


  def check_bounds_and_publish(self, event):
      if self.current_position is None or self.current_velocity is None:
          return

      velocity = deepcopy(self.current_velocity)
      adjusted_velocity = self.spatial_bounds(self.current_position, velocity)

      if adjusted_velocity != velocity:
        self.current_velocity = adjusted_velocity
        self.publish_safe_twist(adjusted_velocity)


  def publish_safe_twist(self, velocity=[0,0,0,0,0,0]):

      velocity = list(velocity)

      # Clamp linear and angular velocities
      velocity[:3] = [self.clamp_velocity(v, -0.15, 0.15) for v in velocity[:3]]
      velocity[3:] = [(self.clamp_velocity(v, -60, 60)) for v in velocity[3:]]

      # Adjust for spatial bounds
      adjusted_velocity = deepcopy(self.spatial_bounds(self.current_position, velocity))

      if adjusted_velocity != velocity:
          # If the adjusted velocity is different, it means the original velocity
          # would take the robot out of bounds so return 
          return
      
      twist_msg = Twist()
      twist_msg.linear_x = velocity[0]
      twist_msg.linear_y = velocity[1]
      twist_msg.linear_z = velocity[2]
      twist_msg.angular_x = math.radians(velocity[3])
      twist_msg.angular_y = math.radians(velocity[4])
      twist_msg.angular_z = math.radians(velocity[5])

      twist_command = TwistCommand()
      twist_command.twist = twist_msg

      self.current_velocity = deepcopy(velocity)
      self.twist_publisher.publish(twist_command)


  def example_send_gripper_close(self, speed=0.1, force=120):
      Robotiq.close(self.robotiq_client, speed, force, block=True)
      rospy.sleep(1)


  def example_send_gripper_open(self):
      Robotiq.open(self.robotiq_client, block=True)
      rospy.sleep(1)

  def example_send_gripper_goto(self, pos=0.1):
      # 0.1 = open. # 0 = close
      Robotiq.goto(self.robotiq_client, pos, speed=0.01, force=10)      
      rospy.sleep(1)      


def main():
  example = ExampleMoveItTrajectories()
  example.reach_named_position("home")  
  example.publish_safe_twist([0.05,0.05,0.05,5,5,5],1)
  rospy.sleep(1)
  example.publish_safe_twist([0,0,0,0,0,0],1)

   
if __name__ == '__main__':
  main()