^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_touch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Fixing boost parse error
* Contributors: Nick Hawes

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
* [scitos_touch] resetting version number.
* Merge branch 'hydro-devel' into dependencies
  Conflicts:
  scitos_door_pass/CMakeLists.txt
  scitos_door_pass/package.xml
  scitos_teleop/CMakeLists.txt
  scitos_teleop/package.xml
* Fixing the maintainer and author entries in package xml files that I am involved in.
* [scitos_touch] Added std_msgs to CMakeLists
* Added license file
* [scitos_touch] Prerelease cleanup
* Using the `${catkin_EXPORTED_TARGETS}` macro in all the add_dependencies statements now.
* ifdef to remove boost join parse error.
* Bug fix:
  Do not use own nodehandle but use getNodeHandle to let rqt take care of your subscribers.
* changed initial motor state assumption to motors are running.
* start with assumption that motors are off in the beginning.
* Added some comments and deleted rubbish from package.xml
* Bug fixing:
  forgot to start the ros:spin thread and had to add a ! for the colour change
  The button is now a bit smaller to be able to use something else besides the button in rqt.
  The colour is now only changed if there is real change in the value.
* Complete restructuring.
  Getting rid of ros_comm because it is not needed and complicates erverything.
* Missing connect function added to give some functionality to the button.
* Remodeling the emergency stop button into an rqt plugin.
  First attempt. Limited functionality.
* Added documentation.
* added scitos_msgs_genccp dependency.
* Fixed dependencies and removed "glob"-ing from CMakeLists to make it clean.
* Button toggle is now based on service success.
* Listening to the /bumper topic to determine the real state of the motors instead of guessing from service success.
* Making Qt stop gracefully when catching sigint or sigterm.
  Calling ros::shutdown when Qt app quits.
* Button now changes colour when pressed and motors are successfully stopped.
* bug fix.
* Toggle between emergency stop and reset motors.
* First working version. Should stop motors on button press. Still not clean. Segfaults when closed because ros::spin is not properly terminated yet.
* Base framework for a touchscreen emergency stop button. No funtionality yet.
* Contributors: Christian Dondrup, Nick Hawes, cdondrup
