cmake_minimum_required(VERSION 2.8.3)
project(scitos_virtual_bumper)
set(CMAKE_BUILD_TYPE Debug)

	find_package(catkin REQUIRED COMPONENTS 
    nav_msgs
    geometry_msgs 
    roscpp 
    rospy 
    scitos_msgs 
    std_msgs
    message_generation 
    )

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)

#add_service_files(
#   FILES
#)

add_message_files(
	FILES 
	virtualbumperevent.msg
	virtualbumperreport.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  geometry_msgs 
)

catkin_package(
	CATKIN_DEPENDS message_runtime
	INCLUDE_DIRS 
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Libraries
# Executables
add_executable(virtual_bumper src/virtual_bumper/virtual_bumper.cpp)

add_dependencies(virtual_bumper ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
target_link_libraries (virtual_bumper ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS virtual_bumper 
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE
    )

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
    
install(PROGRAMS
   scripts/vb_notifications.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
 
#install(DIRECTORY doc
#    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#    )
