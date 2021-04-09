cmake_minimum_required(VERSION 2.8.3)
project(rosbuzz)

# Definition of colors for Message
if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

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
  geometry_msgs
  nav_msgs
  message_generation
)

# Optinal OMPL libs for path planning
if(OMPL_FOUND)
  find_package(ompl)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOMPL_FOUND=1")
  message("${Blue} ####################")
  message("${Blue} #################### =======================================================")
  message("${Blue} #################### FOUND OMPL LIB : COMPILING PATH PLANNER HOOKS FOR BUZZ")
  message("${Blue} #################### =======================================================")
  message("${Blue} ####################")

else()
  message("${Red} ####################")
  message("${Red} #################### =======================================================")
  message("${Red} #################### CANNOT FIND OMPL LIB: PATH PLANNER BUZZ HOOKS DISABLED")
  message("${Red} #################### =======================================================")
  message("${Red} ####################")

endif()

##############################
##############################
add_service_files(
  DIRECTORY msg/srv
  FILES
  navigation_tube.srv
  bool_srv.srv
)

add_message_files(
 FILES
 neigh_pos.msg
 hierarchical_status.msg
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
if(OMPL_FOUND)
  include_directories(${OMPL_INCLUDE_DIRS})
  set(PATH_PLANNER_UTIL_SRC 
  src/path_planner_utils/svg_image.h
  src/path_planner_utils/svg_image.cpp
  src/path_planner_utils/path_existance_checking.h 
  src/path_planner_utils/path_existance_checking.cpp
  )
else()
  set(PATH_PLANNER_UTIL_SRC)
endif()

# Executables 
add_executable(rosbuzz_node src/rosbuzz.cpp
  src/roscontroller.cpp
  src/buzz_utility.cpp
  src/buzzuav_closures.cpp
  src/VoronoiDiagramGenerator.cpp
  src/buzz_update.cpp
  ${PATH_PLANNER_UTIL_SRC}
  )
target_link_libraries(rosbuzz_node ${catkin_LIBRARIES} buzz buzzdbg pthread)
add_dependencies(rosbuzz_node rosbuzz_generate_messages_cpp)

if(OMPL_FOUND)
target_link_libraries(rosbuzz_node ompl)
endif()

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
