cmake_minimum_required(VERSION 2.8.3)
project(rosbuzz)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mavros_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
#CATKIN_DEPENDS message_runtime
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Executables 
add_executable(rosbuzz_node src/rosbuzz.cpp
  src/buzz_utility.cpp
  src/buzzuav_closures.cpp
  src/uav_utility.cpp)
target_link_libraries(rosbuzz_node ${catkin_LIBRARIES} /usr/local/lib/libbuzz.so /usr/local/lib/libbuzzdbg.so -lpthread)

# Executables and libraries for installation to do
install(TARGETS rosbuzz_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

find_package(catkin REQUIRED COMPONENTS roslaunch)
roslaunch_add_file_check(launch)
