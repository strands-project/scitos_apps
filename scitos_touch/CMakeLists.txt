cmake_minimum_required(VERSION 2.8.3)
project(scitos_touch)

set(LIB_NAME rqt_emergency_stop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy scitos_msgs rqt_gui rqt_gui_cpp)

## Include Qt
find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

set(emerency_stop_SRCS
  src/rqt_emergency_stop/emergencystop.cpp
)

set(emerency_stop_HDRS
  include/rqt_emergency_stop/emergencystop.h
)

set(emerency_stop_UIS
  src/rqt_emergency_stop/emergencystop.ui
)

set(emerency_stop_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
   INCLUDE_DIRS ${emerency_stop_INCLUDE_DIRECTORIES}
   LIBRARIES rqt_emergency_stop
   CATKIN_DEPENDS roscpp rospy scitos_msgs rqt_gui rqt_gui_cpp
)

include_directories(
  ${emerency_stop_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

catkin_python_setup()

QT4_WRAP_CPP(QT_MOC_HPP ${emerency_stop_HDRS})
QT4_WRAP_UI(QT_FORMS_HPP ${emerency_stop_UIS})

add_library(${LIB_NAME} ${emerency_stop_SRCS} ${QT_FORMS_HPP} ${QT_MOC_HPP})
add_dependencies(${LIB_NAME} scitos_msgs_gencpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${LIB_NAME}
  ${catkin_LIBRARIES}
  ${QT_LIBRARIES}
  ${QT_QTCORE_LIBRARY} 
  ${QT_QTGUI_LIBRARY}
  ${ROS_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(${LIB_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${LIB_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
 
install(PROGRAMS scripts/rqt_emergency_stop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${LIB_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
