# Robot Arm API Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Basic Movement Functions](#examplar-basic-movement-functions)
3. [Object Interactions](#examplar-object-interactions)
4. [Environment Specific Functions](#examplar-environment-specific-functions)
5. [High-Level Tasks](#examplar-high-level-tasks)
6. [Task Resolution Process](#task-resolution-Process)
7. [Parameters and Safety Information](#parameters-and-safety-information)


## Introduction
This documentation provides detailed instructions for controlling a Kinova Robot Arm. The provided code snippets are ready for direct execution without any additional setup. For instance:

````python
speed = 0.05
self.robot.publish_safe_twist([0, 0, speed, 0, 0, 0])
rospy.sleep(2)
self.robot.publish_safe_twist([0, 0, 0, 0, 0, 0])
````

## Examplar Basic Movement Functions
This section details the fundamental movement functions of the robot arm. Each primitive is designed for precision and safety, providing foundational movements that can be combined for complex tasks.

<!-- ### Basic Movements -->

- #### Query: Go Home 
  *Description:* Sends the robot arm to its default home position.
  ````python
  self.robot.reach_named_position("home")
  ````

- #### Query: Move Up (1 second)
  *Description:* Translates the end-effector vertically at a controlled speed.
  ````python
  speed = 0.05
  self.robot.publish_safe_twist([0,0,speed,0,0,0])
  rospy.sleep(1)
  self.robot.publish_safe_twist([0,0,0,0,0,0])
  ````

- #### Query: Move Left (2 seconds)
  *Description:* Moves the end-effector horizontally to the left.
  ````python
  speed = 0.05
  self.robot.publish_safe_twist([0,speed,0,0,0,0])
  rospy.sleep(2)
  self.robot.publish_safe_twist([0,0,0,0,0,0])
  ````

- #### Query: Move Backward (2 seconds)
  *Description:* Retracts the end-effector backward.
  ````python
  speed = 0.05
  self.robot.publish_safe_twist([-speed,0,0,0,0,0])
  rospy.sleep(2)
  self.robot.publish_safe_twist([0,0,0,0,0,0])
  ````

- #### Query: Move to a Z = 0.2
  *Description:* Moves the end-effector to a specific height.
  ````python
  go_to_point_in_one_axis(self, 0.2, axis='z')
  ````

- #### Query: Move Upside Down
  *Description:* Rotates the end-effector to an upside-down orientation.
  ````python
  self.robot.publish_safe_twist([0,0,0,0,0,-20])
  rospy.sleep(9)
  self.robot.publish_safe_twist([0,0,0,0,0,0])
  rospy.sleep(4)
  self.robot.publish_safe_twist([0,0,0,0,0,20])
  rospy.sleep(9)
  self.robot.publish_safe_twist([0,0,0,0,0,0])
  rospy.sleep(4)
  ````

- #### Query: Go to Orientation [90,0,90]
  *Description:* Orients the end-effector to the specified angles.
  ````python
  target_orientation = [90,0,90]
  go_to_orientation(self, target_orientation, angular_direction="roll"):
  ````

- #### Query: Move to Pose
  *Description:* Moves the arm to a specific pose, allowing modification of one orientation at a time.
  ````python
  target_pose = [0.5, 0.2, 0.3, 180, 0, 90]
  move_to_pose(self.current_pose, target_pose, linear_speed=0.05, angular_speed=10)
  ````

- #### Query: Move through Trajectory
  *Description:* Follows a specified array of positions.
  ````python
  trajectory = [[0.5, 0.2, 0.3], [0.6, 0.1, 0.4]]
  go_through_trajectory(self, trajectory, linear_speed=0.05, angular_speed=10)
  ````

- #### Query: Track Object
  *Description:* Tracks a moving object with the end-effector.
  ````python
  track_object(self, "hand")
  ````

- #### Query: Open End-Effector (EEF)
  *Description:* Opens the gripper.
  ````python
  self.robot.example_send_gripper_open()
  ````

- #### Close End-Effector (EEF)
  *Description:* Closes the gripper.
  ````python
  self.robot.example_send_gripper_close(force=10, speed=0.1)
  ````

- #### Query: Close End-Effector at 1 N
  *Description:* Closes the gripper at 1 newton.
  ````python
  self.robot.example_send_gripper_close(force=1, speed=0.1)
  ````

- #### Query: Open Gripper to 0.05
  *Description:* Opens the gripper to position 0.05.
  ````python
  self.robot.example_send_gripper_goto(pos=0.05)
  ````

- #### Query: Move Gripper to 0.07
  *Description:* Opens the gripper to position 0.07.
  ````python
  self.robot.example_send_gripper_goto(pos=0.07)
  ````

- #### Query: Go to Pose with Target=[0.7,0,0.3,90,0,90]
  *Description:* Moves the end effector to the desired target pose.
  ````python
  go_to_pose_rotate_all_directions(self, target_pose=[0.7,0,0.3,90,0,90])
  ````


## Examplar Object Interactions
  This section demonstrates how to interact with objects using the robot arm. The examples combine motion functions to complete specific tasks.
  
- #### Query: Pick up an object using affordance parameters [0.2, 0, 0.1, 170, 10, 100], starting with an end-effector width of 0.055.
  *Description:* Moves the end-effector to pick up an object based on its affordance parameters, starting from the specified end-effector width.
  ````python
  affordance_eef_to_object(self, target_pose=[0.2, 0, 0.1, 170, 10, 100], start_eef_position=0.055)
  ````


- #### Query: Locate Blue Cup Position
  *Description:* Locates the position of the blue cup.
  ````python
  classes = ["blue cup"]
  update_classes_function(self, classes)
  target_position = get_object_position_with_wait(self, classes[0])
  ````

- #### Query: Scoop given Holding Spoon
  *Description:* moves the end-effector to a predefined coffee location and scoops the coffee.
  ````python
  # always scoop at coffee location, which is fixed
  coffee_location = [0.65,0.15,0.18,90,-55,90]
  success = go_to_pose(self, coffee_location, angular_direction="pitch")
  scoop(self)
  shake(self)
  ````

- #### Query: Go to Coffee
  *Description:* moves the end-effector to a predefined coffee location.
  ````python
  coffee_location = [0.65,0.15,0.18,90,-55,90]
  success = go_to_pose(self, coffee_location, angular_direction="pitch")
  ````

- #### Query: Pour
  *Description:* pour a specified amount of liquid or substance.
  ````python
  amount_to_pour = 20
  pour(self, amount_to_pour)
  ````

- #### Query: Put Down with Force
  *Description:* Places an object down at a specified position with controlled force to ensure correct placement.
  ````python
  put_down_position = [0.4, 0.2, 0.05]
  self.robot.put_down_object_with_force_control(self, put_down_position)
  ````

- #### Query: Empty in Blue Cup
  *Description:* Empties the contents held by the end-effector into the blue cup identified by the vision system.
  ````python
  empty_in_cup(self, "blue cup")
  ````

- #### Query: Locate and Empty in Blue Cup**
  *Description:* Locates the blue cup and then empties the contents held by the end effector into it.
  ````python
  empty_in_cup(self, "blue cup")
  ````

- #### Query: Empty in Red Bowl**
  *Description:* Empties the contents held by the robot's end effector into the red bowl.
  ````python
  empty_in_cup(self, "red bowl")
  ````

- #### Query: Take Item**
  *Description:* Acquires an item from a hand that is in motion.
  ````python
  take_item(self, "hand")
  ````

- #### Query: Give Item
  *Description:* Transfers an item to a hand or recipient.
  ````python
  give_item(self, "hand")
  ````

- #### Query: Take Apple
  *Description:* Acquires an apple from a hand that is in motion.
  *Context:* The hand is currently holding an apple.

  ````python
  take_item(self, "hand")
  ````

- #### Query: Give Apple
  *Description:* Transfers an apple to a hand or recipient.
  *Context:* The end-effector is currently holding an apple.
    ````python
    give_item(self, "hand")
    ````

## Examplar Environment Specific Functions
This section outlines methods for interacting with and managing elements specific to the environment.

- #### Query: Open Door
  *Description:* Opens a door using the specified handle number.
  ````python
  door_type = open_door_handle(self, handle_number=1)
  ````

- #### Query: Close Door
  *Description:* Closes a door using the specified handle number.
  ````python
  close_door(self, handle_number=1)
  ````
- #### Query: Pick up Spoon
  *Description:* Picks up a spoon from the workspace.
  *Context: The robot is not holding a spoon
  ````python
  get_spoon(self)
  ````

- #### Query: Put back Spoon
  *Description:* Puts the spoon back in the workspace.
  *Context: The robot is holding a spoon
  ````python
  return_spoon(self)
  ````

- #### Query: Pick up Kettle
  *Description:* Picks up a kettle from the workspace.
  ````python
  get_kettle(self)
  ````

- #### Query: Go to Kettle and grasp it
  *Description:* Moves to the kettle and grasps it.
  ````python
  get_kettle(self)
  ````

- #### Query: Put back Kettle
  *Description:* Puts the kettle back in the workspace.
  ````python
  return_kettle(self)
  ````

- #### Query: Get Mug in Drawer
  *Description:* Retrieves a mug from a drawer.
  ````python
  get_mug_in_drawer(self)
  ````

- #### Query: Put down Object
  *Description:* Places an object down in the workspace.
  ````python
  put_down_object(self)
  ````

- #### Query: Pour into Green Mug
  *Description:* Pours a liquid into a green mug.
  ````python
  object_name = "green mug"
  track_and_pour(self, object_name, [0,0,0.3])
  ````

- #### Query: Draw a Sunflower
  *Description:* Draws a sunflower shape.
  ````python
  object_name = "sunflower"
  drawing(self, "sunflower")
  ````

## Examplar High-Level Tasks
High-level tasks combine multiple primitives to perform complex operations. Several examples are provided below:

- #### Query: Grasp the blue cup from above 
  *Description:* Moves to and grasp the blue cup from above.
  *Context:*  The image shows a blue cup is in the workspace.*
  
  ````python
  self.robot.example_send_gripper_open()
  object_position = return_object_position(self, "blue cup")
  waypoint1_position = [object_position[0], object_position[1], object_position[2] + 0.2]
  waypoint2_position = object_position
  trajectory = [waypoint1_position, waypoint2_position]
  go_through_trajectory(self, trajectory)
  ````

- #### Query: Pick up the blue cup from above
  *Description:* Moves to and grasp the blue cup from above.
  *Context:* The image shows a blue cup is in the workspace
  
  ````python
  self.robot.example_send_gripper_open()
  object_position = return_object_position(self, "blue cup")
  waypoint1_position = [object_position[0], object_position[1], object_position[2] + 0.2]
  waypoint2_position = object_position
  trajectory = [waypoint1_position, waypoint2_position]
  go_through_trajectory(self, trajectory)
  self.robot.example_send_gripper_close()
  self.robot.publish_safe_twist([0,0,0.05,0,0,0])
  rospy.sleep(1)
  self.robot.publish_safe_twist([0,0,0,0,0,0])
  ````

- #### Query: Move to the red bowl from the front.
  *Description:* Moves to the red bowl from a front-facing position.
  *Context:* The image shows the red bowl is in the workspace
  ````python
  self.robot.example_send_gripper_open()
  object_position = return_object_position(self, "red bowl")
  waypoint1_position = [object_position[0] - 0.2, object_position[1], object_position[2]]
  waypoint2_position = object_position
  trajectory = [waypoint1_position, waypoint2_position]
  go_through_trajectory(self, trajectory)
  ````

- #### Query: Pour 100g of water into the blue cup
  *Description:* Pours 100 grams of water into the blue cup.
  *Context:* The image shows the blue cup is in the workspace
  ````python
  get_kettle(self)
  object_position = return_object_position(self, "blue cup")
  waypoint1_position = [object_position[0], object_position[1], object_position[2] + 0.3]
  trajectory = [waypoint1_position]
  go_through_trajectory(self, trajectory)
  amount_to_pour = 100
  pour(self, amount_to_pour)
  ````

- #### Query: Find a cup and place it down, ready for pouring
  *Description:*  Finds a cup and places it down, ready for pouring.
  *Context:* The image shows a cup is not in the workspace and the workspace contains a drawer.
  ````python
  found_cup = False
  for drawer in draws and not found_cup:
      open_drawer()
      found_cup = get_feedback_from_image_or_user()
      if not found_cup(): close_drawer()
      else:
          grasp_cup()
          put_cup_down()
          close_drawer()
  ````

- #### Query: Pour 20g of water into the blue cup
  *Description:* Pours 20 grams of water into the blue cup.
  *Context:* The image shows the blue cup is in the workspace
  ````python
  get_kettle(self)
  object_position = return_object_position(self, "blue cup")
  waypoint1_position = [object_position[0], object_position[1], object_position[2] + 0.3]
  trajectory = [waypoint1_position]
  go_through_trajectory(self, trajectory)
  amount_to_pour = 20
  pour(self, amount_to_pour)
  ````

- #### Query: Open and close two draws.
  *Description:* Opens and closes two specified draws.
  ````python
  door_type = open_door_handle(self, handle_number=1)
  close_door(self, handle_number=1)
  door_type = open_door_handle(self, handle_number=2)
  close_door(self, handle_number=2)
  ````

- #### Query: Scoop coffee into the purple mug
  *Description:* Picks up a spoon, scoops coffee and transfers it into a purple mug.
  ````python
  get_spoon(self)
  go_to_coffee(self)
  scoop(self)
  shake(self)
  empty_in_cup(self, "purple mug")
  return_spoon(self)
  ````

- #### Query: Draw a shape
  *Description:* Draws a specified shape.
  *Context:* The end-effector is empty and a human is currently holding a pen.
  ````python
  take_item(self, "hand")
  drawing(self)
  give_item(self, "hand")
  ````

## Task Resolution Process
To solve each task effectively, decompose it into relevant steps and utilise the provided code examples to ensure accuracy.

## Parameters and Safety Information

*Action Information:*
To run code, action type is 'run_code' and payload is the code snippet

*Safety Information*
The twist command is [x,y,z,roll,pitch,yaw]. The linear component velocities are in m/s and cannot exceed 0.05. The angular component velocities are in degrees/s and cannot exceed 10.

*Paramaters Available:*
self.current_pose
self.current_position
self.objects