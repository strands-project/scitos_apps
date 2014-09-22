cmake_minimum_required(VERSION 2.8.3)
project(scitos_docking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    actionlib 
    actionlib_msgs 
    image_transport 
    message_generation 
    mongodb_store 
    move_base_msgs 
    roscpp 
    rospy 
    std_msgs 
    scitos_msgs 
    scitos_teleop 
    )

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

# Generate services in the 'srv' folder
add_service_files(
   FILES
   Charging.srv
 )

add_action_files(
		DIRECTORY action 
		FILES Charging.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs 
        actionlib_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES charging
#  CATKIN_DEPENDS image_transport roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(charging
#   src/${PROJECT_NAME}/charging.cpp
# )

## Declare a cpp executable
# add_executable(charging_node src/charging_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
# target_link_libraries(charging_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS charging charging_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_charging.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_library(CCircleDetect src/CCircleDetect.cpp)
add_library(CLightClient src/CLightClient.cpp)
add_library(CDump src/CDump.cpp)
add_library(CTimer src/CTimer.cpp)
add_library(CRawImage src/CRawImage.cpp)
add_library(CTransformation src/CTransformation.cpp)
add_library(CChargingClient src/CChargingClient.cpp)
add_library(CPtuClient src/CPtuClient.cpp)
add_library(CChargingActions src/CChargingActions.cpp)
target_link_libraries(CTransformation ${GSL_LIBRARIES})
target_link_libraries(CTransformation ${catkin_LIBRARIES})
target_link_libraries(CChargingClient ${catkin_LIBRARIES})
target_link_libraries(CChargingActions ${catkin_LIBRARIES} CTimer)
target_link_libraries(CLightClient ${catkin_LIBRARIES})
target_link_libraries(CPtuClient ${catkin_LIBRARIES})


add_executable(visual_charging src/docking.cpp)
add_executable(visual_charging_client src/dockingClient.cpp)
add_executable(simple_move_client src/testMoveClient.cpp)
add_executable(simple_ptu_client src/testPtuClient.cpp)

add_dependencies(CTransformation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(CChargingClient ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(CPtuClient ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_charging_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_charging ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_move_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_ptu_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries (visual_charging  CDump)
target_link_libraries (visual_charging  CLightClient)
target_link_libraries (visual_charging  CTimer)
target_link_libraries (visual_charging  CCircleDetect)
target_link_libraries (visual_charging  CRawImage)
target_link_libraries (visual_charging  CTransformation)
target_link_libraries (visual_charging  CChargingClient)
target_link_libraries (CChargingActions  CLightClient)
target_link_libraries (visual_charging  CChargingActions)

target_link_libraries (visual_charging  gsl)
target_link_libraries (visual_charging  blas)
target_link_libraries (visual_charging  ${catkin_LIBRARIES})
target_link_libraries (visual_charging_client  ${catkin_LIBRARIES})
target_link_libraries (simple_move_client  ${catkin_LIBRARIES})
target_link_libraries (simple_ptu_client  ${catkin_LIBRARIES})
