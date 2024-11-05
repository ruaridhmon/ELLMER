### These are the provided utilities ### 
import numpy as np
from scipy.spatial.transform import Rotation as R
import rospy
import math
import time
from utilities_draw import *

# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
### Useful Functions ### 

def calculate_unit_vector(current_position, object_position):
    """Calculate the unit vector from the current position to the object position, ensuring robustness against division by zero."""
    direction_vector = np.array(object_position) - np.array(current_position)
    norm = np.linalg.norm(direction_vector)
    if norm == 0:
        return np.zeros_like(direction_vector)  # Return a zero vector if the norm is 0 to avoid division by zero
    unit_vector = direction_vector / norm
    return unit_vector

def calculate_distance(object_position, current_position):
    """Calculate the distance in units"""
    distance = np.linalg.norm(np.array(object_position) - np.array(current_position))
    a = np.array(object_position) - np.array(current_position)
    # print(f'a {a}')
    return distance

def go_to_point_velocity(current_position, target_position, speed):
    current_position = np.asarray(current_position)
    target_position = np.asarray(target_position)

    angle = calculate_unit_vector(current_position, target_position)
    velocity = np.concatenate((angle * speed, np.array([0, 0, 0])))

    return velocity

class PIDController:
    def __init__(self, kp, ki, kd, set_point, ramp_rate_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = set_point
        self.prev_error = 0
        self.integral = 0
        self.ramp_rate_limit = ramp_rate_limit
        self.prev_control_signal = 0

    def compute_control(self, measurement):
        error = self.set_point - measurement
        self.integral += error * 0.01
        derivative = (error - self.prev_error) / 0.01

        control_signal = self.kp * error + self.ki * self.integral + self.kd * derivative

        if self.ramp_rate_limit is not None:
            control_signal_change = control_signal - self.prev_control_signal
            control_signal_change = max(-self.ramp_rate_limit, min(self.ramp_rate_limit, control_signal_change))
            control_signal = self.prev_control_signal + control_signal_change
            self.prev_control_signal = control_signal

        self.prev_error = error

        return control_signal
    
def euler_to_rot_matrix(euler_angles_deg):
    """Convert Euler angles (in degrees) to a rotation matrix."""
    return R.from_euler('xyz', euler_angles_deg, degrees=True).as_matrix()

def rot_matrix_to_euler(rot_matrix, degrees=True):
    """Convert a rotation matrix to Euler angles."""
    return R.from_matrix(rot_matrix).as_euler('xyz', degrees=degrees)

def calculate_angular_displacement(current_orientation_euler_deg, target_rotation_euler_deg, angular_direction):

    angular_difference = target_rotation_euler_deg - np.array(current_orientation_euler_deg)
    
    if angular_direction == "roll":
        angular_difference = angular_difference[0]
    if angular_direction == "pitch":
        angular_difference = angular_difference[1]
    if angular_direction == "yaw":
        angular_difference = angular_difference[2]

    return angular_difference

def go_to_pose_velocity(current_pose, target_pose, linear_speed=0.05, angular_speed=10, angular_direction='pitch'):
    current_angles = current_pose[3:]
    target_angles = target_pose[3:]
    if angular_direction == "roll":
        current_angles = [current_angles[0],0, 0]
        target_angles =  [target_angles[0],0, 0]
    if angular_direction == "pitch":
        current_angles = [0, current_angles[1], 0]
        target_angles =  [0, target_angles[1], 0]
    if angular_direction == "yaw":
        current_angles = [0, 0, current_angles[2]]
        target_angles =  [0, 0, target_angles[2]]

    angular_velocity = calculate_desired_velocity(current_angles, target_angles, angular_speed)
    linear_velocity = go_to_point_velocity(current_pose[:3], target_pose[:3], linear_speed)

    velocity = np.concatenate([linear_velocity[:3], angular_velocity])

    return velocity

def calculate_desired_velocity(current_orientation_euler_deg, target_rotation_euler_deg, angular_speed_deg_per_sec):

    angular_difference = target_rotation_euler_deg - np.array(current_orientation_euler_deg)
    magnitude = np.linalg.norm(angular_difference)

    if magnitude > 0:
        unit_vector = angular_difference / magnitude
    else:
        unit_vector = angular_difference 

    velocity_command_deg = unit_vector*angular_speed_deg_per_sec

    velocity_command_deg = np.array([velocity_command_deg[0],velocity_command_deg[2],-velocity_command_deg[1]])
    return velocity_command_deg

def get_object_position_with_wait(self, object_name="blue cup"):
    update_time = 0.1

    while True:
        target_position = self.objects.get(object_name, None)

        if target_position is not None:
            break

        self.robot.publish_safe_twist([0, 0, 0, 0, 0, 0])
        rospy.sleep(update_time)

    return target_position

# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
### General Motor Primitives ### 

def go_to_point_in_one_axis(self, target_position, axis='z'):

    axis_index = {'x': 0, 'y': 1, 'z': 2}.get(axis.lower())
    if axis_index is None:
        rospy.logerr(f"Invalid axis: {axis}. Must be 'x', 'y', or 'z'.")
        return False

    distance = target_position - self.current_position[axis_index]
    speed = 0.04

    while abs(distance) > 0.02:
        direction = np.sign(distance)
        velocity = np.zeros(6)
        velocity[axis_index] = direction * speed

        self.robot.publish_safe_twist(velocity)
        rospy.sleep(0.1)

        distance = target_position - self.current_position[axis_index]

    self.robot.publish_safe_twist([0,0,0,0,0,0])

    return True


def go_to_pose(self, target_pose, angular_speed=10, linear_speed=0.05, angular_direction="pitch"):

    if len(target_pose) == 3:
        target_pose = target_pose + [0,0,0]
        angular_direction = None
    else:
        target_pose = target_pose

    update_time = 0.1
    not_at_pose = True
    
    while not_at_pose:
        velocity = go_to_pose_velocity(self.current_pose, target_pose, linear_speed, angular_speed, angular_direction)
        distance_linear = calculate_distance(self.current_position, target_pose[:3])
        if angular_direction:
            angular_difference = calculate_angular_displacement(self.current_pose[3:], target_pose[3:], angular_direction)
        else: 
            angular_difference = 0

        if distance_linear < 0.03:
            velocity[:3] = 0

        if abs(angular_difference) < 2:
            velocity[3:] = 0

        if distance_linear < 0.03 and abs(angular_difference) < 2:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            not_at_pose = False

        self.robot.publish_safe_twist(velocity)
        rospy.sleep(update_time)

    return not_at_pose

def go_to_position(self, target_position):
    update_time = 0.1
    speed = 0.06

    target_position = list(target_position)

    while calculate_distance(self.current_position, target_position) >= 0.03:
        velocity = go_to_point_velocity(self.current_position, target_position, speed)
        self.robot.publish_safe_twist(velocity)
        rospy.sleep(update_time)

    self.robot.publish_safe_twist([0, 0, 0, 0, 0, 0])

    return True

def go_to_orientation(self, target_orientation, angular_direction="roll"):
    # [90, 0, 90] is home

    target_pose = [0,0,0] + target_orientation
    update_time = 0.1
    angular_speed=10
    linear_speed=0.05

    not_at_pose = True
    while not_at_pose:
        velocity = go_to_pose_velocity(self.current_pose, target_pose, linear_speed, angular_speed, angular_direction)
        if angular_direction:
            angular_difference = calculate_angular_displacement(self.current_pose[3:], target_pose[3:], angular_direction)
        else: 
            angular_difference = 0

        print(f'angular_difference {angular_difference}')

        # only move angular
        velocity[:3] = 0

        if abs(angular_difference) < 5:
            angular_speed=1

        if abs(angular_difference) < 0.5:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            not_at_pose = False

        print(f'velocity {velocity}')
        self.robot.publish_safe_twist(velocity)
        rospy.sleep(update_time)

    self.robot.publish_safe_twist([0,0,0,0,0,0])
    return not_at_pose

def go_through_trajectory(self, trajectory, linear_speed=0.05, angular_speed=10):
    for pose in trajectory:
        go_to_pose(self, pose, angular_direction=None)
        
    return True       

def update_classes_function(self, updated_classes):
    self.update_classes(updated_classes)
    rospy.sleep(1)


def return_object_position(self, object_name):
    update_classes_function(self, [object_name])
    object_position = get_object_position_with_wait(self, object_name)
    object_position = list(object_position)
    print(f'object_position {object_position}')
    return object_position

# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
### Environment Specific but reasonably General Motion Primitives ###

def track_object(self, object_name, offset=[0,0,0]):
    update_time = 0.1
    speed = 0.05
    update_position = True
    target_position = None

    classes = [object_name]
    update_classes_function(self, classes)    

    while True:
        if target_position is None or update_position:
            target_position = get_object_position_with_wait(self, object_name)
            target_position = [target_position[0] + offset[0], target_position[1] + offset[1], target_position[2] + offset[2]]

        distance = calculate_distance(self.current_position, target_position)

        print(f'distance {distance}')

        if abs(distance) < 0.03:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
        else:
            velocity = go_to_point_velocity(self.current_position, target_position, speed)
            self.robot.publish_safe_twist(velocity)

        rospy.sleep(update_time)


def take_item(self, object_name="hand"):
    self.robot.reach_named_position("home")
    self.robot.example_send_gripper_goto(pos=0.055)
    transfer_item(self, object_name=object_name)
    # take item
    self.robot.publish_safe_twist([0,0,0,0,0,0])
    self.robot.example_send_gripper_close()
    self.robot.reach_named_position("home")

def give_item(self, object_name="hand"):
    self.robot.reach_named_position("home")
    transfer_item(self, object_name=object_name)
    # pass item
    self.robot.publish_safe_twist([0,0,0,0,0,0])
    self.robot.example_send_gripper_open()
    self.robot.reach_named_position("home")


def transfer_item(self, object_name="hand"):
    # this function tracks hand with EEF open at an offset and then closes
    # when the hand is close enough for enough time 

    self.robot.publish_safe_twist([0,0,0,0,0,0])
    rospy.sleep(0.1)
    update_time = 0.1
    linear_speed = 0.03
    angular_speed = 4
    target_position = None
    angular_direction = "yaw"

    classes = [object_name]
    update_classes_function(self, classes)    

    time_close = 0     
    close_duration_threshold = 0.4
    stop_short_distance = 0.2   

    while True:
        hand_position = get_object_position_with_wait(self, object_name)
        hand_position = [hand_position[0], hand_position[1], hand_position[2]]

        fixed_reference_point = [0.65,0,hand_position[2]]
        direction_vector_xy = [hand_position[0] - fixed_reference_point[0], hand_position[1] - fixed_reference_point[1]]
        
        # Normalize the 2D direction vector to get a unit vector in the XY plane
        magnitude_xy = (direction_vector_xy[0]**2 + direction_vector_xy[1]**2) ** 0.5
        unit_vector_xy = [d / magnitude_xy for d in direction_vector_xy]
        
        target_position_xy = [hand_position[i] - unit_vector_xy[i] * stop_short_distance for i in range(2)]
        target_position = target_position_xy + [hand_position[2]]  # Keeps the hand's Z position
        
        dy = hand_position[1] - fixed_reference_point[1] #self.current_position[1]
        dx = hand_position[0] - fixed_reference_point[0] #self.current_position[0]
        angle_rad = np.arctan2(dy, dx)  # Angle in radians
        angle_deg = np.degrees(angle_rad)  # Convert to degrees
        yaw_orientation = angle_deg + 90  # Adjusting the yaw orientation            
        target_pose = target_position + [90,0,yaw_orientation]

        distance = calculate_distance(self.current_position, target_position)
        angular_difference = calculate_angular_displacement(self.current_pose[3:], target_pose[3:], angular_direction)

        print(f'distance {distance} angular_difference {angular_difference}')

        if distance < 0.1:
            linear_speed = 0.03
        else:
            linear_speed = 0.06

        if abs(angular_difference) < 5:
            angular_speed = 2
        else: 
            angular_speed = 4


        if abs(distance) < 0.05 and abs(angular_difference) < 3:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            if time_close == 0:  # Start the timer
                time_close = time.time()

            if (time.time() - time_close) >= close_duration_threshold:
                print('take item')
                break

        else:
            time_close = 0 
            velocity = go_to_pose_velocity(self.current_pose, target_pose, linear_speed=linear_speed, angular_speed=angular_speed, angular_direction=angular_direction)

            if abs(angular_difference) < 3:
                velocity[3:] = 0

            self.robot.publish_safe_twist(velocity)

        rospy.sleep(update_time)

    return 

def put_down_object(self):
    # uses force control for initial contact the assumes constant z
    put_down_position_above = [0.65, -0.02, 0.2]

    go_to_position(self, put_down_position_above)
    force_t = self.wrench_global.force.y
    count = 0
    while True:
        print(f'self.latest_avg_force_y {self.wrench_global.force.y}')

        change_in_force = self.wrench_global.force.y - force_t
        print(f'change_in_force {change_in_force}')
        force_t = self.wrench_global.force.y

        if abs(change_in_force) > 1:
            count += 1
            if count == 2:
                self.robot.publish_safe_twist([0,0,0,0,0,0])
                self.robot.example_send_gripper_goto(pos=0.07)
                self.robot.publish_safe_twist([0,0,0.05,0,0,0])
                rospy.sleep(2)
                self.robot.publish_safe_twist([0,0,0,0,0,0])              
                break
        else:
            self.robot.publish_safe_twist([0,0,-0.05,0,0,0])
            count = 0

        rospy.sleep(0.1)


def drawing(self):

    speed = 0.01
    update_time = 0.1

    rospy.sleep(0.4)
    waypoints = return_waypoints()
    # assumed plate is at x = 0.4, y = 0. Can make this dynamic if desired.
    waypoints = [(x + 0.4, y, 0) for x, y in waypoints]

    first_waypoint =  waypoints[0]
    go_to_pose(self, [first_waypoint[0],first_waypoint[1],0.23,180,0,90],angular_direction="roll")

    previous_z_force = self.wrench_global.force.z
    force_difference = 0
    count = 0
    while abs(force_difference) < 0.3 or count < 5:
        count += 1
        self.robot.publish_safe_twist([0,0,-0.02,0,0,0])
        force_difference = self.wrench_global.force.z - previous_z_force
        previous_z_force = self.wrench_global.force.z
        print(f'force_difference {force_difference}')
        rospy.sleep(update_time)

    self.robot.publish_safe_twist([0,0,0.02,0,0,0])
    rospy.sleep(2.1*update_time)
    self.robot.publish_safe_twist([0,0,0,0,0,0])

    for waypoint in waypoints:
        waypoint = list(waypoint)
        waypoint[2] = 0
        while True:
            current_position = [self.current_position[0],self.current_position[1],0]
            velocity = go_to_point_velocity(current_position, waypoint, speed)
            velocity[2] = 0

            self.robot.publish_safe_twist(velocity)

            if calculate_distance(current_position, waypoint) < 0.001:
                self.robot.publish_safe_twist([0, 0, 0, 0, 0, 0])
                break

            rospy.sleep(update_time)

    self.robot.publish_safe_twist([0,0.03,0.03,0,0,0])
    rospy.sleep(5)
    self.robot.publish_safe_twist([0,0,0,0,0,0])     

# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------
### Environment Specific Motion Primitives ###

# ----------------------------------------------------------------------
# Opening and Closing Doors

def return_handle_position(handle_number=1):

    if handle_number == 1: 
        target_position = [0.945, 0.12755799293518066, 0.11246903985738754]
    if handle_number == 2:
        target_position = [0.935, 0.14132400810718536, 0.3378752863407135]

    return target_position

def return_handle_position_close(handle_number=1):
    '''
    assume the handle position are detected 
    '''
    if handle_number == 1: 
        target_position = [0.8290873765945435, 0.12755799293518066, 0.11246903985738754]
    if handle_number == 2:
        target_position = [0.93, 0.39, 0.3349169194698334]

    return target_position

def close_door(self, handle_number=1):
    target_position = return_handle_position_close(handle_number)
    if handle_number == 1:
        go_through_trajectory(self, [target_position])
        rospy.sleep(0.5)
        self.robot.example_send_gripper_close()
        # push until get resistance or back at start
        while abs(self.wrench_global.force.x) < 30 and (abs(self.current_position[0] - self.door_start_point[0]) > 0.005):
            self.robot.publish_safe_twist([0.03,0,0,0,0,0])

        self.robot.publish_safe_twist([0,0,0,0,0,0])
        self.robot.example_send_gripper_goto(pos=0.06)
        self.robot.publish_safe_twist([-0.03,0,0,0,0,0])
        rospy.sleep(3)
        self.robot.publish_safe_twist([0,0,0,0,0,0])

    if handle_number == 2:
        waypoint1_position = [target_position[0] - 0.2, target_position[1] - 0.2, target_position[2]]
        waypoint2_position = [target_position[0] - 0.2, target_position[1], target_position[2]]
        waypoint3_position = target_position
        waypoint4_position = [target_position[0], target_position[1]-0.2, target_position[2]]
        waypoint5_position = [target_position[0]-0.1, target_position[1]-0.2, target_position[2]]
        trajectory = [waypoint1_position, waypoint2_position, waypoint3_position, waypoint4_position, waypoint5_position]
        go_through_trajectory(self, trajectory)



def open_door_handle(self, handle_number=1):
    self.robot.example_send_gripper_goto(pos=0.06)
    target_position = return_handle_position(handle_number)
    waypoint1_position = [target_position[0] - 0.1, target_position[1], target_position[2]]
    waypoint2_position = target_position
    trajectory = [waypoint1_position, waypoint2_position]
    go_through_trajectory(self, trajectory)
    self.robot.example_send_gripper_close()
    self.door_start_point = self.current_position
    open_door(self, target_position)


def open_door(self, handle_position):
    self.pid_y = PIDController(kp=1, ki=0.01, kd=0.2, set_point=0, ramp_rate_limit=0.001)
    self.pid_torque_x = PIDController(kp=60, ki=0.2, kd=0.1, set_point=0, ramp_rate_limit=0.09)

    start_position = self.current_position[0]

    while True:
        control_y = self.pid_y.compute_control(self.wrench_global.force.z)
        control_y = max(min(control_y, 0.05), -0.05)

        control_torque_y = self.pid_torque_x.compute_control(self.wrench_global.torque.y)
        control_torque_y = max(min(control_torque_y, 4), -4)

        angle_difference = (self.current_pose[5] - 90)
        speed_x = -0.003*abs(math.cos(math.radians(abs(angle_difference))))

        force_x = self.wrench_global.force.x

        if abs(force_x) > 25: speed_x = 0
        pulled_distance = self.current_position[0] - start_position

        if abs(angle_difference) < 5 and abs(pulled_distance) > 0.025:
            speed_pull = -0.05
            if abs(force_x) > 20: speed_pull = 0
            velocity = np.array([speed_pull,0,0,0,0,0])
        else:
            velocity = np.array([speed_x, control_y, 0, 0, -control_torque_y, 0])

        if abs(angle_difference) > 28: #32
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            self.robot.example_send_gripper_goto(pos=0.055)            
            if angle_difference < 0:
                door_type = 'hinge_left'
            else: 
                door_type = 'hinge_right'
            break

        if abs(pulled_distance) > 0.09 and abs(angle_difference) < 5:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            self.robot.example_send_gripper_goto(pos=0.055)
            door_type = 'pulled'
            break        

        if abs(pulled_distance) > 0.2:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            self.robot.example_send_gripper_goto(pos=0.055)            
            door_type = '???'
            break        

        rospy.sleep(0.01)

        self.robot.publish_safe_twist(velocity)

    if 'hinge' in door_type:
        open_hinge_fully(self, handle_position, door_type)

    return door_type        

def open_hinge_fully(self, handle_position, door_type):
    """
    Motion primitive just defined for the sepcific left hinge. Straight forward to add a right
    """

    if door_type == "hinge_left":
        self.robot.publish_safe_twist([-0.05,0,0,0,0,0])
        rospy.sleep(1.5)        
        linear_speed = 0.04
        angular_speed = 4
        target_pose = [handle_position[0]-0.15,handle_position[1]-0.16,handle_position[2],90,0,90]
        angular_direction = 'yaw'
        go_to_pose(self, target_pose, angular_speed, linear_speed, angular_direction)
        target_pose[0] += 0.15
        go_to_pose(self, target_pose, angular_speed, linear_speed, angular_direction)
        self.robot.publish_safe_twist([-0.03,0.04,0,0,0,0])
        rospy.sleep(5)
        self.robot.publish_safe_twist([0.03,-0.05,0,0,0,0])
        rospy.sleep(0.5)
    self.robot.publish_safe_twist([0,0,0,0,0,0])

   
# -----------------------------------------------------------
# Scooping, shaking and emptying

def scoop(self):
    self.robot.publish_safe_twist([0,0.025,0.04,0,0,-17])
    rospy.sleep(3)
    self.robot.publish_safe_twist([0,-0.01,-0.06,0,0,12])
    rospy.sleep(3)
    self.robot.publish_safe_twist([0,-0.015,0.02,0,0,6])
    rospy.sleep(3)
    self.robot.publish_safe_twist([0,-0.01,0.0,0,0,0])
    rospy.sleep(3)
    self.robot.publish_safe_twist([0,0,0.0,0,0,0])
    rospy.sleep(1)


def shake(self):
    self.robot.publish_safe_twist([0,0.04,0,0,0,0])
    rospy.sleep(0.2)
    self.robot.publish_safe_twist([0,-0.04,0,0,0,0])
    rospy.sleep(0.2)
    self.robot.publish_safe_twist([0,0.08,0,0,0,0])
    rospy.sleep(0.2)
    self.robot.publish_safe_twist([0,-0.08,0,0,0,0])
    rospy.sleep(0.2)
    self.robot.publish_safe_twist([0,0.09,0,0,0,0])
    rospy.sleep(0.2)
    self.robot.publish_safe_twist([0,-0.09,0,0,0,0])
    rospy.sleep(0.2)
    self.robot.publish_safe_twist([0,0,0.0,0,0,0])
    rospy.sleep(1)

def empty_in_cup(self, object):
    classes = [object]
    update_classes_function(self, classes)

    angular_speed = 15
    linear_speed = 0.05
    angular_direction="pitch"

    update_time = 0.1
    not_at_pose = True

    # first move cup to given height
    target_orientation = [90,-67,90]
    
    go_to_orientation(self, target_orientation, angular_direction="pitch")
    go_to_point_in_one_axis(self, 0.44, axis='z')

    while not_at_pose:

        target_position = get_object_position_with_wait(self, object)
        target_pose = [target_position[0]-0.07,target_position[1]-0.19,(target_position[2]+0.18),90,-65,90]

        velocity = go_to_pose_velocity(self.current_pose, target_pose, linear_speed, angular_speed, angular_direction)
        distance_linear = calculate_distance(self.current_position, target_pose[:3])
        angular_difference = calculate_angular_displacement(self.current_pose[3:], target_pose[3:], angular_direction)

        if distance_linear < 0.05:
            velocity[:3] = 0

        if abs(angular_difference) < 2:
            velocity[3:] = 0

        if distance_linear < 0.05 and abs(angular_difference) < 2:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            not_at_pose = False

        self.robot.publish_safe_twist(velocity)
        rospy.sleep(update_time)

    self.robot.publish_safe_twist([0,0.045,0.04,0,0,-20])
    rospy.sleep(3) 
    self.robot.publish_safe_twist([0,0,0,0,0,0])

    return True

# ---------------------------------------------------------------------------
# pouring


def track_and_pour(self, object_name, offset=[0,0,0], amount_to_pour=90):
    update_time = 0.1
    speed = 0.06
    update_position = True
    target_position = None

    offset = [-0.05,-0.2,0.15]
    finish_once_close = False
    update_object_threshold = None

    classes = [object_name]
    update_classes_function(self, classes)    

    print('is this printing?')
    time_close = 0     
    close_duration_threshold = 1    

    while True:
        if target_position is None or update_position:
            target_position = get_object_position_with_wait(self, object_name)
            target_position = [target_position[0] + offset[0], target_position[1] + offset[1], target_position[2] + offset[2]]

        distance = calculate_distance(self.current_position, target_position)

        if update_object_threshold is not None and distance < update_object_threshold:
            update_position = False

        if abs(distance) < 0.03:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            if time_close == 0:  # Start the timer
                time_close = time.time()

            if (time.time() - time_close) >= close_duration_threshold:
                print('pouring')
                break

        else:
            time_close = 0 
            velocity = go_to_point_velocity(self.current_position, target_position, speed)
            self.robot.publish_safe_twist(velocity)

        rospy.sleep(update_time)

    pour(self, amount_to_pour)        


def pour(self, amount_to_pour):
    force_up = self.avg_force_y
    force_difference = 0

    force_up = self.avg_force_y       
    start = time.time()

    self.robot.publish_safe_twist([0,0,0,0,0,-4])

    while force_difference < amount_to_pour and (time.time() - start) < 18:
        
        self.robot.publish_safe_twist([0,0,0,0,0,-4])

        force_difference = abs(self.avg_force_y - force_up)*1000/9.81
        print(f'force_difference {force_difference}')
        rospy.sleep(0.1)
    print(f'force_difference {force_difference}')
    t = time.time() - start
    print(f'time {t}')

    self.robot.publish_safe_twist([0,0,0,0,0,0])
    rospy.sleep(0.1)
    roll = self.current_pose[4]
    time_to_vertical = abs(roll) / abs(30)
    self.robot.publish_safe_twist([0,0,0,0,0,30])
    rospy.sleep(time_to_vertical)
    self.robot.publish_safe_twist([0,0,0,0,0,0])

    return True    


# -----------------------------------------------------------------
# pick and place 
def go_to_coffee(self):
    coffee_pose = [0.57,0.16,0.18,90,-55,90]
    success = go_to_pose(self, [coffee_pose[0],coffee_pose[1],coffee_pose[2]+0.2])
    success = go_to_pose(self, coffee_pose, angular_direction="pitch")


def go_to_spoon_from_above(self,return_spoon=False):
    target_position = [0.75,0.27,0.1516015326976776]
    if return_spoon:
        target_position[2] += 0.03
    waypoint1_position = [target_position[0], target_position[1], target_position[2] + 0.2]
    waypoint2_position = [target_position[0], target_position[1], target_position[2]]
    speed = 0.05
    trajectory = [waypoint1_position, waypoint2_position]
    go_through_trajectory(self, trajectory)    


def get_spoon(self):
    self.robot.example_send_gripper_goto(pos=0.03)  
    go_to_spoon_from_above(self)
    self.robot.example_send_gripper_close()
    # this code clears container after grasping spoon as always need to do this
    self.robot.publish_safe_twist([0,0,0.05,0,0,0])
    rospy.sleep(4)
    self.robot.publish_safe_twist([0,0,0,0,0,0])

def return_spoon(self):
    go_to_spoon_from_above(self, return_spoon=True)
    self.robot.example_send_gripper_goto(pos=0.05)
    # this code clears container after grasping spoon as always need to do this
    self.robot.publish_safe_twist([0,0,0.05,0,0,0])
    rospy.sleep(4)
    self.robot.publish_safe_twist([0,0,0,0,0,0])

def get_kettle(self):
    self.robot.reach_named_position("home")
    target_position = [0.64,-0.37,0.162]
    self.robot.example_send_gripper_goto(pos=0.03)    
    waypoint1_position = [target_position[0] - 0.15, target_position[1], target_position[2] + 0.2]
    waypoint2_position = [target_position[0] - 0.15, target_position[1], target_position[2]]
    waypoint3_position = [target_position[0], target_position[1], target_position[2]]
    trajectory = [waypoint1_position, waypoint2_position, waypoint3_position]
    go_through_trajectory(self, trajectory)    
    self.robot.example_send_gripper_close()
    rospy.sleep(0.35)
    self.kettle_closed_position = self.current_position
    print(f'self.kettle_closed_position {self.kettle_closed_position}')
    self.robot.publish_safe_twist([0,0,0.05,0,0,0])
    rospy.sleep(5)
    self.robot.publish_safe_twist([0,0,0,0,0,0])
    print('done1')

def return_kettle(self):
    target_position = self.kettle_closed_position
    waypoint1_position = [target_position[0], target_position[1], target_position[2] + 0.2]
    waypoint2_position = [target_position[0], target_position[1], target_position[2]]
    trajectory1 = [waypoint1_position, waypoint2_position]
    go_through_trajectory(self, trajectory1)    
    self.robot.example_send_gripper_goto(pos=0.03)
    self.robot.publish_safe_twist([0,0,0,0,0,0]) 
    rospy.sleep(0.4)
    waypoint3_position = [target_position[0]-0.2, target_position[1], target_position[2]]
    waypoint4_position = [target_position[0]-0.2, target_position[1], target_position[2] + 0.2]
    trajectory2 = [waypoint3_position, waypoint4_position]
    go_through_trajectory(self, trajectory2)
    self.robot.publish_safe_twist([0,0,0,0,0,0])
    self.robot.reach_named_position("home")
    print('done2')    

    
def get_mug_in_drawer(self):
    target_position = [1.038, 0.12, 0.30007025599479675]
    self.robot.example_send_gripper_goto(pos=0.055)
    waypoint1_position = [target_position[0] - 0.15, target_position[1], target_position[2]]
    waypoint2_position = target_position
    trajectory = [waypoint1_position, waypoint2_position]
    go_through_trajectory(self, trajectory)
    self.robot.example_send_gripper_close()
    self.robot.publish_safe_twist([-0.04,0,0.0,0,0,0])
    rospy.sleep(6)
    self.robot.publish_safe_twist([0,0,0,0,0,0])

# ------------------------------------------------------------------
# Rotating in all directions

def go_to_pose_velocity_rotate_all_directions(current_pose, target_pose, linear_speed=0.05, angular_speed=10):
    current_angles = current_pose[3:]
    target_angles = target_pose[3:]

    angular_velocity = calculate_desired_velocity(current_angles, target_angles, angular_speed)
    linear_velocity = go_to_point_velocity(current_pose[:3], target_pose[:3], linear_speed)

    velocity = np.concatenate([linear_velocity[:3], angular_velocity])

    return velocity

def calculate_angular_displacement_all_directions(current_orientation_euler_deg, target_rotation_euler_deg):

    angular_difference = target_rotation_euler_deg - np.array(current_orientation_euler_deg)
    # Compute the Euclidean distance of the angular difference
    angular_distance = np.linalg.norm(angular_difference)
    
    return angular_distance

def go_to_pose_rotate_all_directions(self, target_pose=[0.3,0,0.1,90,0,90], angular_speed=5, linear_speed=0.05):

    if len(target_pose) == 3:
        target_pose = target_pose + [0,0,0]
    else:
        target_pose = target_pose

    update_time = 0.1
    not_at_pose = True
    
    angular_speed = 12
    linear_speed =0.05
    while not_at_pose:

        velocity = go_to_pose_velocity_rotate_all_directions(self.current_pose, target_pose, linear_speed, angular_speed)
        distance_linear = calculate_distance(self.current_position, target_pose[:3])
        angular_distance = calculate_angular_displacement_all_directions(self.current_pose[3:], target_pose[3:])
        print(f'angular_distance {angular_distance}')

        if distance_linear < 0.1:
            linear_speed =0.03

        if abs(angular_distance) < 13:
            angular_speed = 4

        if distance_linear < 0.03:
            velocity[:3] = 0

        if abs(angular_distance) < 4:
            velocity[3:] = 0

        if distance_linear < 0.03 and abs(angular_distance) < 4:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            not_at_pose = False

        self.robot.publish_safe_twist(velocity)
        rospy.sleep(update_time)

    return not_at_pose



def go_to_pose_rotate_all_directions_dynamic(self, object_name="hand", angular_speed=5, linear_speed=0.05):

    self.robot.publish_safe_twist([0,0,0,0,0,0])
    rospy.sleep(0.1)
    update_time = 0.1
    linear_speed = 0.03
    angular_speed = 4
    target_position = None

    classes = [object_name]
    update_classes_function(self, classes)    

    time_close = 0     
    close_duration_threshold = 0.4

    while True:
        hand_position = get_object_position_with_wait(self, object_name)
        print(f'hand_position {hand_position}')

        target_position = hand_position
        target_pose = list(target_position) + [170,30,100]

        distance = calculate_distance(self.current_position, target_position)
        angular_distance = calculate_angular_displacement_all_directions(self.current_pose[3:], target_pose[3:])

        if distance < 0.1:
            linear_speed = 0.03
        else:
            linear_speed = 0.06

        if abs(angular_distance) < 6:
            angular_speed = 3
        else: 
            angular_speed = 8

        if abs(distance) < 0.05 and abs(angular_distance) < 3:
            self.robot.publish_safe_twist([0,0,0,0,0,0])
            if time_close == 0:  # Start the timer
                time_close = time.time()

            if (time.time() - time_close) >= close_duration_threshold:
                print('take item')
                # break

        else:
            time_close = 0 
            velocity = go_to_pose_velocity_rotate_all_directions(self.current_pose, target_pose, linear_speed, angular_speed)

            if abs(angular_distance) < 3:
                velocity[3:] = 0

            self.robot.publish_safe_twist(velocity)

        rospy.sleep(update_time)
    return 



def affordance_eef_to_object(self, target_pose=[0.2, 0, 0.1, 170, 10, 100], start_eef_position=0.055):
    """
    (1) move EEF to eef_start_pose
    (2) go_to_pose 
    (3) close EEF
    """
    self.robot.example_send_gripper_goto(pos=start_eef_position)    
    go_to_pose_rotate_all_directions(self, target_pose)
    self.robot.example_send_gripper_close(speed=0.02)


def affordance_eef_to_hand(self, target_pose=[0.2, 0, 0.1, 170, 10, 100], start_eef_position=0.055):
    """
    (1) move EEF to eef_start_pose
    (2) go_to_pose 
    (3) close EEF
    """
    go_to_pose_rotate_all_directions(self, target_pose)
    self.robot.example_send_gripper_close(speed=0.02) 