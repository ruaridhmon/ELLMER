#! /usr/bin/env python3

import rospy
import IPython
import numpy
import os
import subprocess
import netft_rdt_driver.srv as srv

def record(iteration=0, recordTime=10):
    zeroFTSensor()
    rospy.sleep(2)

    # Create a folder for the bag
    bagName = 'i{}'.format(iteration)
    # dir_save_bagfile = 'tests/'
    dir_save_bagfile = '/home/kinova/dev/kinova/catkin_workspace/src/netft_rdt_driver/scripts/tests'
    if not os.path.exists(dir_save_bagfile):
        os.makedirs(dir_save_bagfile)

    topics = ["/netft/netft_data"]
    subprocess.Popen('rosbag record -q -O {} {}'.format(bagName, " ".join(topics)), shell=True, cwd=dir_save_bagfile)   
    rospy.sleep(1)
    rospy.sleep(recordTime)
        
    # Stop recording rosbag
    terminate_ros_node("/record")
    rospy.sleep(1)

def zeroFTSensor():
    zeroSensorRos = rospy.ServiceProxy('/netft/zero', srv.Zero)
    rospy.wait_for_service('/netft/zero', timeout=0.5)
    zeroSensorRos()

def terminate_ros_node(s):
    list_cmd = subprocess.Popen("rosnode list", shell=True, stdout=subprocess.PIPE)
    list_output = list_cmd.stdout.read()
    retcode = list_cmd.wait()
    assert retcode == 0, "List command returned %d" % retcode

    for string in list_output.decode('utf-8').split("\n"):
        if string.startswith(s):
            os.system("rosnode kill " + string)

    # for string in list_output.split("\n"):
    #     if string.startswith(s):
    #         os.system("rosnode kill " + string)

if __name__ == '__main__':
    rospy.init_node("netft")
    IPython.embed()
