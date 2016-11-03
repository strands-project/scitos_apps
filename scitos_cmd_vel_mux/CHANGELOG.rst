^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_cmd_vel_mux
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
