cmake_minimum_required(VERSION 2.8.3)
project(ptu_follow_frame)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_generation  
  tf
)

# catkin_python_setup()

add_service_files(
  FILES
  StartFollowing.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ptu_follow_frame
#  CATKIN_DEPENDS geometry_msgs rospy sensor_msgs
#  DEPENDS system_lib
)

# include_directories(
#  ${catkin_INCLUDE_DIRS}
# )

#############
## Install ##
#############

install(PROGRAMS
  scripts/ptu_follow.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ptu_follow_frame.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
