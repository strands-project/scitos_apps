^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_teleop
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2016-11-03)
------------------

0.1.0 (2016-05-24)
------------------

0.0.23 (2016-05-03)
-------------------

0.0.22 (2015-12-10)
-------------------

0.0.21 (2015-12-07)
-------------------

0.0.20 (2015-05-22)
-------------------

0.0.19 (2015-05-22)
-------------------

0.0.18 (2015-05-17)
-------------------

0.0.16 (2015-03-19)
-------------------

0.0.15 (2015-03-18)
-------------------
* Added resetting the magnetic barrier motor stop to rumble_control
* Contributors: Christian Dondrup

0.0.14 (2014-12-17)
-------------------

0.0.13 (2014-11-21)
-------------------

0.0.12 (2014-11-20)
-------------------

0.0.11 (2014-11-20)
-------------------

0.0.10 (2014-11-19)
-------------------

0.0.9 (2014-11-11)
------------------

0.0.8 (2014-11-09)
------------------

0.0.7 (2014-11-08)
------------------
* final and tested version of loader
* machine tags for cmd_vel_mux and teleop
* Contributors: Jaime Pulido Fentanes

0.0.6 (2014-11-06)
------------------

0.0.5 (2014-10-23)
------------------

0.0.4 (2014-10-13)
------------------

0.0.3 (2014-09-22)
------------------

0.0.2 (2014-09-22)
------------------

0.0.1 (2014-09-22)
------------------
* Merging somehow removed the std_msgs.
* Merge branch 'hydro-devel' into dependencies
  Conflicts:
  scitos_door_pass/CMakeLists.txt
  scitos_door_pass/package.xml
  scitos_teleop/CMakeLists.txt
  scitos_teleop/package.xml
* Some more merging.
* Merge branch 'hydro-devel' into teleop
  Conflicts:
  scitos_teleop/CMakeLists.txt
  scitos_teleop/package.xml
* Fixing the maintainer and author entries in package xml files that I am involved in.
* remove from package.xml as well.
* [scitos_teleop] CMakeLists was missing dependecies specified in package.xml. std_msgs not used.
* Installing the udev rule directly won't work properly. Following the example of http://wiki.ros.org/kobuki_ftdi now, using a script to do it.
  This can be run with `rosrun scitos_teleop create_udev_rules`
* Added license file
* Prerelease cleanup
  * the udev rule has been renamed to identify it's origin.
  * the udev rule will be automatically installed and therefore /dev/input/rumblepad is now the default.
* Moved action_buttons message to scitos_teleop package.
* Enabling the scitos services again.
  In the beginning of the project the scitos_msgs where hidden in the scitos drivers repository which made it necessary to check for the existence of the package during compile time because the joystick should also work in simulation where there is not scitos_drivers repository.
  Once the scitos_msgs where moved to scitos_common, I removed this check from the CMakeLists but forgot to remove the compiler flag from the code. Therefore the rumble_control was always compiled without these service enabled.
  This is fixed now.
* fixing machine tags for scitos_teleop
* adding machine tags to cmd_vel_mux, teleop_joystick and scitos docking launch files
* Using the `${catkin_EXPORTED_TARGETS}` macro in all the add_dependencies statements now.
* added scitos_apps_msgs as dependency
* add joy as dependency
* Docking position injection improvement.
* Joystick now allows to control forward acceleration by cross buttons (axis 7) and speed by stick (axis 1)
* Actually passing along the js argument from every top-level launch file to the core launch file.
  Closing issue `#26 <https://github.com/strands-project/scitos_apps/issues/26>`_
* Update README.md
  Removed very old and confusing set-up instructions and replaced with link to strands_systems readme file. Also added a link to cmd_vel_mux so that people know about it.
* Update README.md
  Update troubleshooting because of: https://github.com/strands-project/autonomous_patrolling/issues/33
* Using new EyeLids JointState to move eye lids in sync.
* Some urgent restructuring and adding of dependencies to make it build again.
* Update teleop_joystick_mux.launch
  Remove call to cmd_vel_mux to keep it more general usable.
* Added implementation of cmd_vel_mux to allow arbitration. To use run scitos_teleop_mux.launch and have the navigation publish to /cmd_vel_mux/input/navigation instead of /cmd_vel.
  Joystick will now get priority over navigation if dead-man-switch is pressed.
* adapting to the changes by cburbridge
* fix scitos_msgs depend
* Fixed a bug with the message generation.
* Updated Readme file
* Updated cheat sheet.
* Bugfix. Forgot a self.
* Bugfi for the zero position button. Now works if inversed as well.
* Now the rumbepad control just sends button messages when they change.
  The head can be turned backwards if you hit Y.
* Updated and created new launch files to start just the core functionalities, just head, just base and all.
* The head and eye control are now in the safe python script and not seperated anylonger.
* Seperated the rumblepad_control and teleop_base.
  rumbelpad_control now takes responsibility of republishing the joy msg if the pad is in the right mode and the deadman switch is pressed. It also provides the emergency stop and motor reset functionalities.
  teleop_base just transformes the joystick mesages from teleop_joystick/joy published by the rumblepad_control to veleocities for /cmd_vel
* fixing messed up file
* Using parameter for joystick device
* Using parameter for joystick device
* Setting joysick to system independent name
* Now the robot also moves the eyes when moving the head with the joystick and when pressing the top right shoulder button it sets the head position to 0 0.
* Moved the aaction button message to scitos_apps_msgs.
* rumblepad_control is now checking if the pad runs in teh correct mode to prevent confusion because of a wrong button layout.
* Now publishing a custom button message containing 4 bools, one for every action button.
* Created a message for the action buttons
* Changed namespace to teleop_joystick
* Update README.md
  Some troubleshooting info added.
* Added a readme to explain baisc installation and usage of the scitos_teleop package.
* Adapted file structure to match pr2_teleop.
  Changed CMakeLists.txt and package.xml to reflect the new name scitos_teleop.
  Renamed rumblepad_control.launch to teleop_joystick.launch and changed the names according to new package name.
* Moved rumblepad_control to scitos_teleop/src
* Contributors: Bruno Lacerda, Chris Burbridge, Christian Dondrup, Jaime Pulido Fentanes, Marc Hanheide, Tom Krajnik, cburbridge, cdondrup
