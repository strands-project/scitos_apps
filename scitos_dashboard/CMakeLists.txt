cmake_minimum_required(VERSION 2.8.3)
project(scitos_dashboard)

find_package(catkin REQUIRED COMPONENTS 
    roslib 
    rospy 
    rqt_gui 
    rqt_robot_dashboard 
    scitos_msgs 
    std_msgs
    )

catkin_python_setup()
catkin_package()

## Install files to right locations
find_package(class_loader)
#class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#install(TARGETS ${LIB_NAME}
  #ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  #LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/scitos_dashboard #Script to run it standalone
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#install(DIRECTORY include/${LIB_NAME}/
  #DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#)
