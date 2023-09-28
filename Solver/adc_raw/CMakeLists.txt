cmake_minimum_required(VERSION 3.0.2)
project(adc_raw)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
)

 add_message_files(
   FILES
  Mcp3208_data.msg
 )

generate_messages(
  DEPENDENCIES
  std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy std_msgs message_runtime roscpp
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

add_executable(adc_raw_node src/adc_node.cpp src/adc_functions.cpp src/MCP_3208_functions.cpp)
target_link_libraries(adc_raw_node ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(adc_raw_node adc_raw_generate_messages_cpp)
#add_dependencies(adc_raw_node adc_raw_EXPORTED_TARGETS)

add_definitions(-lwiringPi)
target_link_libraries(adc_raw_node ${wiringPi_LIB} -lwiringPi -lwiringPiDev -lpthread -lm -lcrypt -lrt)

## Mark executables and/or libraries for installation
install(TARGETS adc_raw_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
