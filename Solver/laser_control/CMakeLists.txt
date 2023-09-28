cmake_minimum_required(VERSION 3.0.2)
project(laser_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch config scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_package()

add_executable(led_control_node src/led_control_node.cpp src/led_control_functions.cpp)
target_link_libraries(led_control_node ${catkin_LIBRARIES})

add_executable(laser_control_node src/laser_control_node.cpp src/laser_control_functions.cpp)
target_link_libraries(laser_control_node ${catkin_LIBRARIES})

add_definitions(-lwiringPi)
target_link_libraries(led_control_node ${wiringPi_LIB} -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt)