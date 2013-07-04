### Setting up the workspace
This is just a simple example set-up and might differ from yours.
* Create a ros workspace: `mkdir -p ros-ws/src && cd ros-ws/src`
* Clone the github repository: `git clone https://github.com/strands-project/scitos_apps.git`
* Setting up the catkin workspace: in ros-ws/src run `catkin_init_workspace`
  * Change to the root directory of the repository which is ros-ws in our case.
  * Skip this if not installed on robot: To enable the emergency stop and motor reset ability the scitos_mira package has to be installed and sourced before making the autonomous_patrolling package: `source <path_to_according_workspace>/scitos_mira/devel/setup.bash`. If you are using the simulator and not the real robot, you do not have to source the scitos_mira package. This will prevent the emergency stop and bumper reset buttons from working. All the other functionalities will work as described.
  * Run `catkin_make` in ros-ws (catkin_make builds all binary files and creates environment variables like the setup.bash)
  * Troubleshooting: It might complain about missing include directories. If this is the case, go to the specified folder and create an empty include directory.
Now everything should be built and you go to the next part which describes the usage of the scitos_teleop package.

### Scitos teleop
The scitos_teleop package is designed to work with a Logitech Wireless Gamepad F710.
* Source the corresponding setup.bash: `source <your_catkin_workspace>/devel/setup.bash`
* Launch the rumblepad control: `roslaunch scitos_teleop teleop_joystick.launch`
 * If the simulator or scitos_mira drivers are running, you should now be able to control the robot using the joypad.
* Controlling the robot (if you do not press any buttons, the rumbelpad control will not interfere with any autonomous behaviour but can be used to emergency stop the robot or reset the bumper after a crash): You can find a cheat sheet in the doc directory of scitos_teleop.
 * Moving the robot: press and hold the dead man switch -> top left shoulder button, move the robot with the left joystick or D-Pad (toggel between those with the "Mode" button).
 * Emergency stop: lower left sholder button. Has to be pressed down completely. Be aware that the gamepad is turning itself off after a certain time and that this button does not turn it on automatically. You have to press one of the other buttons in order to turn it back on.
 * Freerun: "Back" button on the pad. (Move robot around manually)
 * Reset/Re-enable motors: "Start" button on the pad. (Use after emergency stop or bumper crash)
 * Move the head: use right joystick
 * Move eye lids: use lower right shoulder button
