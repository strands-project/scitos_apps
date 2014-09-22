^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_cmd_vel_mux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.3 (2014-09-22)
------------------

0.0.2 (2014-09-22)
------------------

0.0.1 (2014-09-22)
------------------
* [scitos_cmd_vel_mux] Set version number to 0.0.0 to have a common start.
* Added license to package.xml
* Release preparations
* adding machine tags to cmd_vel_mux, teleop_joystick and scitos docking launch files
* Renamed topics of comd_vel_mux to match the old ones
  Also changed the name to yocs_cmd_vel_mux in package.xml to allow automated installation.
* Changed the name of the cmd_vel_mux in launch file to new version
* updated dependency: package got renamed under hydro
* Added README file with basic instructions.
  Updated package.xml to make rosdep work.
* Added implementation of cmd_vel_mux to allow arbitration. To use run scitos_teleop_mux.launch and have the navigation publish to /cmd_vel_mux/input/navigation instead of /cmd_vel.
  Joystick will now get priority over navigation if dead-man-switch is pressed.
* Contributors: Christian Dondrup, Jaime Pulido Fentanes, Lars Kunze, cdondrup
