cmake_minimum_required(VERSION 2.8.3)
project(scitos_teleop)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    geometry_msgs
    message_generation 
    roscpp 
    rospy 
    scitos_msgs
    sensor_msgs
    std_msgs
    )

add_message_files(
  FILES
  action_buttons.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(rumblepad_control
#   src/${PROJECT_NAME}/rumblepad_control.cpp
# )

## Declare a cpp executable
add_executable(rumble_control src/rumble_control.cpp)
add_executable(teleop_base src/base_teleop.cpp)

add_dependencies(rumble_control ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(teleop_base ${catkin_EXPORTED_TARGETS})

target_link_libraries(rumble_control ${catkin_LIBRARIES})
target_link_libraries(teleop_base ${catkin_LIBRARIES})

#############
## Install ##
#############

install(PROGRAMS
  scripts/head_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/create_udev_rules
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS rumble_control teleop_base
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY udev 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
