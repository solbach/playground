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

# Min. PCL 1.7.2 due to ProgressiveMorphologicalFilter
find_package(PCL 1.7.2 REQUIRED)

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
