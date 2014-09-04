cmake_minimum_required(VERSION 2.8.3)
project(scitos_teleop)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs sensor_msgs scitos_apps_msgs scitos_msgs)
#catkin_python_setup()
catkin_package()

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(rumble_control src/rumble_control.cpp)
add_executable(teleop_base src/base_teleop.cpp)

add_dependencies(rumble_control ${catkin_EXPORTED_TARGETS})
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

install(TARGETS rumble_control teleop_base
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY doc
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES udev/73-scitos-teleop-persistent-joystick.rules
  DESTINATION /etc/udev/rules.d
)

