cmake_minimum_required(VERSION 2.8.3)
project(scitos_docking)

find_package(catkin REQUIRED COMPONENTS 
    actionlib 
    actionlib_msgs 
    dynamic_reconfigure
    geometry_msgs
    image_transport 
    message_generation 
    mongodb_store 
    move_base_msgs 
    nav_msgs
    roscpp 
    rospy 
    sensor_msgs
    scitos_msgs 
    scitos_teleop 
    std_msgs
    tf 
    )

include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules(GSL REQUIRED gsl)
include_directories(${GSL_INCLUDE_DIRS})
link_directories(${GSL_LIBRARY_DIRS})

add_service_files(
   FILES
   Charging.srv
)

add_action_files(
   DIRECTORY action 
   FILES Charging.action
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
)

catkin_package(
   INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

# Libraries
add_library(CCircleDetect src/scitos_docking/CCircleDetect.cpp)
add_library(CLightClient src/scitos_docking/CLightClient.cpp)
add_library(CDump src/scitos_docking/CDump.cpp)
add_library(CTimer src/scitos_docking/CTimer.cpp)
add_library(CRawImage src/scitos_docking/CRawImage.cpp)
add_library(CTransformation src/scitos_docking/CTransformation.cpp)
add_library(CChargingClient src/scitos_docking/CChargingClient.cpp)
add_library(CPtuClient src/scitos_docking/CPtuClient.cpp)
add_library(CChargingActions src/scitos_docking/CChargingActions.cpp)

add_dependencies(CTransformation ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(CChargingClient ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(CPtuClient ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries(CTransformation ${GSL_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(CChargingClient ${catkin_LIBRARIES})
target_link_libraries(CChargingActions ${catkin_LIBRARIES} CTimer CLightClient)
target_link_libraries(CLightClient ${catkin_LIBRARIES})
target_link_libraries(CPtuClient ${catkin_LIBRARIES})

# Executables
add_executable(visual_charging src/scitos_docking/docking.cpp)
add_executable(visual_charging_client src/scitos_docking/dockingClient.cpp)
add_executable(simple_move_client src/scitos_docking/testMoveClient.cpp)
add_executable(simple_ptu_client src/scitos_docking/testPtuClient.cpp)

add_dependencies(visual_charging_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(visual_charging ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_move_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(simple_ptu_client ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

target_link_libraries (visual_charging 
    CDump 
    CLightClient 
    CTimer 
    CCircleDetect
    CRawImage
    CTransformation
    CChargingClient
    CChargingActions
    gsl
    blas
    ${catkin_LIBRARIES}
    )
target_link_libraries (visual_charging_client  ${catkin_LIBRARIES})
target_link_libraries (simple_move_client  ${catkin_LIBRARIES})
target_link_libraries (simple_ptu_client  ${catkin_LIBRARIES})

#############
## Install ##
#############

install(TARGETS 
        CCircleDetect 
        CLightClient 
        CDump 
        CTimer 
        CRawImage 
        CTransformation 
        CChargingClient 
        CPtuClient 
        CChargingActions 
        visual_charging 
        visual_charging_client 
        simple_move_client 
        simple_ptu_client
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

install(DIRECTORY doc
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
