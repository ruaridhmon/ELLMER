cmake_minimum_required(VERSION 2.8.3)
project(netft_rdt_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  diagnostic_updater
  diagnostic_msgs
  message_generation
  geometry_msgs  # Ensure this is included  
  actionlib
  actionlib_msgs
)

find_package(
  Boost REQUIRED COMPONENTS system thread program_options
)

#add_definitions(-DBOOST_LOG_DYN_LINK=1)


add_service_files(DIRECTORY srv FILES
  Zero.srv
  ZeroAxis.srv  # Add your ZeroAxis.srv file here  
)

# for actionlib
add_action_files(
    DIRECTORY action
    FILES Zero.action
)
generate_messages(
    DEPENDENCIES actionlib_msgs
    geometry_msgs  # Add geometry_msgs if your messages use geometry types    
)
catkin_package(
  CATKIN_DEPENDS actionlib_msgs
  message_runtime  # Ensure message_runtime is included here  
)

message("Boost_LIBRARIES " ${Boost_LIBRARIES})
message("Boost_INCLUDE_DIRS " ${Boost_INCLUDE_DIRS})
include_directories( include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

# Driver library that can be used by other stuff (ie : netft_ethercat_hardware) 
add_library(${PROJECT_NAME} src/netft_rdt_driver.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ) 
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)

# Stand-alone ROS node that uses driver library 
add_executable(netft_node src/netft_node.cpp)
target_link_libraries(netft_node ${PROJECT_NAME}  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(netft_node ${PROJECT_NAME} ${netft_rdt_driver_EXPORTED_TARGETS})
