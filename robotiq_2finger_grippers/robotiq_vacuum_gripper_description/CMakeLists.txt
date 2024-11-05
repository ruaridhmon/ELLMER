cmake_minimum_required(VERSION 3.0.2)
project(robotiq_vacuum_gripper_description)

find_package(catkin REQUIRED COMPONENTS
  xacro
)

catkin_package()

file(GLOB xacro_files urdf/*.urdf.xacro)
xacro_add_files(${xacro_files} TARGET urdf INSTALL)

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
