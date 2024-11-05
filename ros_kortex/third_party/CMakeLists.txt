
cmake_minimum_required(VERSION 3.0.2)
project(third_party)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_python_setup()

catkin_package()