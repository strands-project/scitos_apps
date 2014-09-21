cmake_minimum_required(VERSION 2.8.3)
project(scitos_ptu)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
    actionlib
    actionlib_msgs
    rospy 
    sensor_msgs 
    )


add_action_files(DIRECTORY action 
  FILES PanTilt.action PtuGoto.action PtuReset.action PtuSetVel.action)
generate_messages(DEPENDENCIES actionlib_msgs sensor_msgs)

catkin_package(
)


