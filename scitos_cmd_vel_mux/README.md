### Installation
* Run `catkin_make`
* Source the environment
* Run rosdep
```
rosdep install scitos_cmd_vel_mux
```

### Usage
* Run
```
roslaunch scitos_cmd_vel_mux mux.launch
```
* Remap your navigation stack /cmd_vel output to /cmd_vel_mux/input/navigation
* Run
```
roslaunch scitos_teleop_mux.launch
```
This remaps the joystick output to /cmd_vel_mux/input/joystick and now the joystick will always have priority as soon as you press the dead-man-switch.
