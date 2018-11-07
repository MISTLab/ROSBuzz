cmake_minimum_required(VERSION 2.8.3)
project(rosbuzz)

if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSIMULATION=${SIM} -DMAVROSKINETIC=${KIN}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
  sensor_msgs
  nav_msgs
  message_generation
)

##############################
##############################

add_message_files(
 FILES
 neigh_pos.msg
)

generate_messages(
DEPENDENCIES
sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES xbee_ros_node
  CATKIN_DEPENDS roscpp std_msgs mavros_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include ${rosbuzz_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
# set the path to the library folder
link_directories(/usr/local/lib)

# Executables 
add_executable(rosbuzz_node src/rosbuzz.cpp
  src/roscontroller.cpp
  src/buzz_utility.cpp
  src/buzzuav_closures.cpp
  src/VoronoiDiagramGenerator.cpp
  src/buzz_update.cpp)
target_link_libraries(rosbuzz_node ${catkin_LIBRARIES} buzz buzzdbg pthread)
add_dependencies(rosbuzz_node rosbuzz_generate_messages_cpp)

# Executables and libraries for installation to do
install(TARGETS rosbuzz_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)
