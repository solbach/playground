cmake_minimum_required(VERSION 2.8.3)
project(playground)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
)

catkin_package()

###########
## Build ##
###########

include_directories(src ${catkin_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(playground src/playground.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(playground
   ${catkin_LIBRARIES}
)
