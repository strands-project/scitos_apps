cmake_minimum_required(VERSION 2.8.3)
project(scitos_touch)

set(LIB_NAME rqt_emergency_stop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
    scitos_msgs 
    std_msgs
    roscpp 
    rospy 
    rqt_gui 
    rqt_gui_cpp)

## Include Qt
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

## Set sources and headers
set(emerency_stop_SRCS
  src/rqt_emergency_stop/emergencystop.cpp
)

set(emerency_stop_HDRS
  include/rqt_emergency_stop/emergencystop.h
)

set(emerency_stop_UIS
  src/rqt_emergency_stop/emergencystop.ui
)

## Set include directories
set(emerency_stop_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR} #Needed for genrated moc headers
)

## Specify everything important for the catkin package
catkin_package(
   INCLUDE_DIRS ${emerency_stop_INCLUDE_DIRECTORIES}
   LIBRARIES rqt_emergency_stop
   CATKIN_DEPENDS roscpp rospy scitos_msgs rqt_gui rqt_gui_cpp
)

## Specify include directories
include_directories(
  ${emerency_stop_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

## Activate python setup to be able to install rqt plugin with setup.py
catkin_python_setup()

## Wrap Qt cpp and ui files
QT4_WRAP_CPP(QT_MOC_HPP ${emerency_stop_HDRS})
QT4_WRAP_UI(QT_FORMS_HPP ${emerency_stop_UIS})

## Create a library and not an executable because this is going to be a plugin
add_library(${LIB_NAME} ${emerency_stop_SRCS} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(${LIB_NAME} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY}
  ${ROS_LIBRARIES}
)


## Install files to right locations
find_package(class_loader)
class_loader_hide_library_symbols(${LIB_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_emergency_stop #Script to run it standalone
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${LIB_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
