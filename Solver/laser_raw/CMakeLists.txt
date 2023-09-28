cmake_minimum_required(VERSION 3.0.2)
project(laser_raw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  adc_raw
)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS roscpp rospy std_msgs message_runtime 
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

install(DIRECTORY launch config scripts src
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(laser_raw_node src/laser_node.cpp src/laser_functions.cpp)
add_definitions(-lwiringPi)
target_link_libraries(laser_raw_node ${wiringPi_LIB} -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt)
target_link_libraries(laser_raw_node ${catkin_LIBRARIES})
add_dependencies(laser_raw_node ${catkin_EXPORTED_TARGETS})
