# ELLMER

This repository provides the source code for a robotic framework that integrates language models, a curated knowledge base and real time force and visual feedback for a Kinova robot. The system is designed to enable robots to perform complex tasks in unpredictable environments. A demonstration video illustrating the system's capabilities can be viewed [here](https://www.youtube.com/watch?v=WiLEw9Zu2MA). The project is organised into the following components:

## Project Components

### 1. netft_rqt_driver

The `netft_rqt_driver` folder is built upon the [netft_rdt_driver repository](https://github.com/joaomoura24/netft_rdt_driver) and provides a ROS interface for the ATI Network Force/Torque Sensor System. This interface is crucial for accurate calibrating and real-time management of force and torque data, enabling the robot to interact with its environment with precision.

#### Calibration Procedure

To calibrate the sensor, follow these steps:

1. **Move the robotic arm to its home position.** In this position, the force sensor's x-axis points upward (opposing gravity), the y-axis extends to the right (from the robot's front view), and the z-axis projects outward from the robot.

2. **Calibrate the sensor in the home position** using the following ROS service call:

    ```bash
    rosservice call /netft/zero_axis "axis: 'all'"
    ```

3. **Rotate the sensor 90 degrees around its z-axis** until the x-axis is parallel to the ground.

4. **Calibrate the sensor again** to remove the influence of gravity on the force measurements by executing the following command:

    ```bash
    rosservice call /netft/zero_axis "axis: 'x'"
    ```

This procedure ensures that the force readings remain accurate and (mostly) unaffected by gravitational forces when the arm is in motion.

### 2. robotiq_2finger_gripper

This folder contains the control interface for the Robotiq 2-finger gripper, based on the [robotiq_2finger_grippers repository](https://github.com/ipab-slmc/robotiq_2finger_grippers). It is designed to facilitate precise manipulation and gripping tasks using the Robotiq 2-finger gripper.

### 3. ROS Kortex Integration

#### 3.1 Kortex Drivers and Customisation

The ROS Kortex integration is facilitated through the official [ROS Kortex package](https://github.com/Kinovarobotics/ros_kortex), which provides the necessary drivers and interfaces to control the Kinova robotic arm. Custom code is primarily located in the `ros_kortex/kortex_examples/src/ELLMER` directory, providing enhanced functionalities for Kinova robotic arms. This includes a utilities file that the language module can access, enabling the robot to operate with real-time feedback and respond dynamically to its environment.


**Note:**
- TargetCoordinates and TargetCoordinatesArray msg were added to the kortex driver
- Ensure the shebang (#!) line at the top of each Python file is correctly configured to point to your Python environment, ensuring the scripts run properly in your setup.

#### 3.2 Vision Module - Grounded-Segment-Anything

The vision capabilities are integrated through the [Grounded-Segment-Anything module](https://github.com/IDEA-Research/Grounded-Segment-Anything), contained within the `third_party` package of the ROS Kortex folder. This module is crucial for object detection and grasp coordination.

#### Key Components:

- **Kinovavision**: Contains additional files for vision processing, including JSON files for camera-robot transformation matrices and Python scripts for calibration and vision processing (`kinova_calibrate.py` and `kinova_vision.py`). The `kinova_vision.py` script is responsible for object tracking, segmentation, and publishing target coordinates to the `target_coordinates` topic. It is based on the 'grounded_sam_demo.py' file in the [Grounded-Segment-Anything module](https://github.com/IDEA-Research/Grounded-Segment-Anything) and can be found in ros_kortex/third_party/Grounded-Segment-Anything/EfficientSAM.

### Launching the Project

To start the project components, launch the following ROS nodes:

```bash
roslaunch kortex_driver kortex_driver.launch
roslaunch kortex_examples run_vision.launch
roslaunch kortex_examples run_force.launch
roslaunch kortex_examples run_brain.launch
```

These commands will start the kortex drivers, vision processing, force feedback, and the main project execution loop.

Next, ensure that the AWS server is running to enable communication between the custom GPT model and the robot. For more details, refer to the LLM Robot Planning repository.

### 4. LLM Robot Planning

This module generates robot action plans based on user queries through natural language commands. Leveraging the RAG-LLM (Retrieval-Augmented Generation using a Large Language Model) approach, it interprets user input to generate code that controls robot commands, drawing from a knowledge base that can be expanded. The commands can be processed and sent to the robot for execution.

Code is provided to establish a feedback loop between the high-level reasoning of the LLM and low-level sensorimotor control. This section also evaluates the effectiveness of RAG-augmented plan generation compared to non-RAG methods, assessing the quality and adaptability of the robot's responses.

## Installation Guide
The system has been tested on an Ubuntu 20.04 platform with an RTX 2080 GPU and ROS-Noetic. Follow these steps for installation:


    sudo apt install python3 python3-pip
    sudo python3 -m pip install conan==1.59
    conan config set general.revisions_enabled=1
    conan profile new default --detect > /dev/null
    conan profile update settings.compiler.libcxx=libstdc++11 default
    mkdir -p catkin_workspace/src
    cd catkin_workspace/src
    git clone https://github.com/ruaridhmon/ELLMER.git
    cd ../
    rosdep install --from-paths src --ignore-src -y

Next, navigate to the Grounded-Segment-Anything directory within the 'third_party' package and follow the installation steps outlined in the [original repository](https://github.com/IDEA-Research/Grounded-Segment-Anything?tab=readme-ov-file):


Finally, build the project:

    catkin_make
    source devel/setup.bash


### Vision Calibration

**Note:** Ensure the intrinsic parameters are updated for your specific camera model. To calibrate the camera, run:

```bash
roslaunch run_calibration.launch
```

### Camera Intrinsics
To retrieve the intrinsic parameters for the Azure Kinect Camera, refer to the instructions in this [Open3D issue](https://github.com/isl-org/Open3D/issues/1183) or use the [Azure Kinect ROS driver](https://github.com/microsoft/Azure_Kinect_ROS_Driver). After launching the camera, to get the intrinsic values, run:

```bash
rostopic list | grep camera_info
rostopic echo /depth_to_rgb/camera_info
```

### AprilTag

For our setup, an April tag was used to calibrate the camera with the robot base: https://github.com/AprilRobotics/apriltag. We used the "tagStandard41h12" tag printed on on A3 paper for increased accuracy, though other tags can be used as well.

## Hardware Setup Overview 
The Kinova flange was used to connect the force sensor to the Robotiq-140 gripper. The reference design and CAD files are available [here](https://artifactory.kinovaapps.com/artifactory/generic-documentation-public/Documentation/Gen3/CADS%20%26%20Drawings/End_Effector_Reference_Design.zip). <p>The setup used in the paper can be seen <a href="images/set-up.png" target="_blank">here</a>.</p>


## Additional Questions and Issues
If you have any issues or questions, you can contact Ruaridh Mon-Williams at rumon@mit.edu or ruaridh.mw@ed.ac.uk. 