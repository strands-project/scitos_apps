### Setting up the workspace
Please follow instructions given here: https://github.com/strands-project/strands_systems/blob/master/README.md

### Scitos teleop
The scitos_teleop package is designed to work with a Logitech Wireless Gamepad F710.
* Source the corresponding setup.bash: `source <your_catkin_workspace>/devel/setup.bash`
* Launch the rumblepad control: `roslaunch scitos_teleop teleop_joystick.launch`
 * If the simulator or scitos_node is running, you should now be able to control the robot using the joypad.
 * Please also have look at: https://github.com/strands-project/scitos_apps/tree/master/scitos_cmd_vel_mux which represents a nice way of giving the joystick priority over the navigation stack.
* Controlling the robot (if you do not press any buttons, the rumbelpad control will not interfere with any autonomous behaviour but can be used to emergency stop the robot or reset the bumper after a crash): You can find a cheat sheet in the doc directory of scitos_teleop.
 * Dead-Man-Switch: top left shoulder button, keep pressed to move robot or head.
 * Moving the robot: move the robot with the left joystick or D-Pad (toggel between those with the "Mode" button).
 * Emergency stop: lower left sholder button. Has to be pressed down completely. Be aware that the gamepad is turning itself off after a certain time and that this button does not turn it on automatically. You have to press one of the other buttons in order to turn it back on.
 * Freerun: "Back" button on the pad. (Move robot around manually)
 * Reset/Re-enable motors: "Start" button on the pad. (Use after emergency stop or bumper crash)
 * Move the head including eyes: use right joystick.
 * Move head to zero position: top right shoulder button.
 * Turn head 180 degrees: Y button.
 * Move eye lids: use lower right shoulder button.
 
### Troubleshooting
If you get a message like: ```[ERROR] [1372933726.815471480]: Couldn't open joystick /dev/.... Will retry every second.``` 
you have to export the joystick device, e.g.: `export JOYSTICK_DEVICE=/dev/input/js1` and start the launch file again.
* Using udev: 
 * You can also make sure that the joystick will always be found if you use the udev rule provided: Copy `scitos_apps/scitos_teleop/udev/73-persistent-joystick.rules` to `/etc/udev/rules.d/`
 * Now just export the joystick variable: `export JOYSTICK_DEVICE=/dev/input/rumblepad` or add this to your `.bashrc` and source it
 * You might have to restart the udev service or log out and back in again before this shows any effect.
