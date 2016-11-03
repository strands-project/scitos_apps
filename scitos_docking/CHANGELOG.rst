^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_docking
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.1 (2016-11-03)
------------------
* Merge pull request `#157 <https://github.com/strands-project/scitos_apps/issues/157>`_ from gestom/hydro-devel
  Docking ends up in a 'safe' position
* Docking end up far away from station in the case of a timeout
* Safe timeout
* Contributors: Christian Dondrup, Nick Hawes, Tom Krajnik

0.1.0 (2016-05-24)
------------------
* Merge pull request `#156 <https://github.com/strands-project/scitos_apps/issues/156>`_ from gestom/hydro-devel
  Docking now supports up to 4 different stations
* Station identification distance increased
* Position injection when undocking
* Incorporated in calibration
* Docking now recognises 4 different stations. Docking is backwards compatible.
* Contributors: Nick Hawes, Tom Krajnik

0.0.23 (2016-05-03)
-------------------
* Depth image decoding fixed.
* Position injection precision increased by resolving transform bug
* Contributors: Christian Dondrup, Tom Krajnik

0.0.22 (2015-12-10)
-------------------
* reverting df7aea03f367933eb8171b8930cabce60349bc0a
  as discussed in https://github.com/strands-project/scitos_apps/commit/df7aea03f367933eb8171b8930cabce60349bc0a#commitcomment-14897218 and https://github.com/strands-project/strands_systems/pull/114#issuecomment-163432083 this needs to be reverted.
* Contributors: Marc Hanheide

0.0.21 (2015-12-07)
-------------------
* Fixing Chaos.
  Actually adapting the topic name to the change in openni2
* Contributors: Karl Review

0.0.20 (2015-05-22)
-------------------
* Merge branch 'hydro-devel' of https://github.com/strands-project/scitos_apps into dyn_subscription
* keep subscribed to most topics
* Contributors: Marc Hanheide

0.0.19 (2015-05-22)
-------------------
* Revert "(un)subscribe from all the topics"
* (un)subscribe from all the topics
* Contributors: Marc Hanheide

0.0.18 (2015-05-17)
-------------------
* Adding position injection to launch file.
  Closes `#138 <https://github.com/strands-project/scitos_apps/issues/138>`_
* Contributors: Christian Dondrup

0.0.16 (2015-03-19)
-------------------
* Avoiding string-conversion-related warnings.
* Contributors: Tom Krajnik

0.0.15 (2015-03-18)
-------------------
* Docking station position parametrized - allows to use maps with arbitrary docking station position. Circle detection sensitivity increased - allows to dock in adverse lighting conditions. Initial stages of docking do not check if the robot behaves as expected anymore (this was included to verify if other nodes still send commands to the robot or if the robot was moving when charging was initiated) - improves interaction with higher navigation layers that seemed to activate the charging while the robot was still moving.
* Contributors: Tom Krajnik

0.0.14 (2014-12-17)
-------------------

0.0.13 (2014-11-21)
-------------------

0.0.12 (2014-11-20)
-------------------
* change the status message for head on and head off
* removed Port0_12V_Enabled
* removed Port0_12V_Enabled
* Revert "Revert "Improvements to charging""
* Contributors: Marc Hanheide

0.0.11 (2014-11-20)
-------------------
* Revert "Improvements to charging"
* Contributors: Marc Hanheide

0.0.10 (2014-11-19)
-------------------
* Light behaviour improved, head restart optional.
* Head swithing off when charging is now optional
* Obstacle check when undock, head restart, PTU facing backwards when charging.
* Now checking for obstacles during undocking
* Contributors: Jaime Pulido Fentanes, Tom Krajnik

0.0.9 (2014-11-11)
------------------
* Make turning on/off the light's EBC configurable.
* Contributors: lucasb-eyer

0.0.8 (2014-11-09)
------------------

0.0.7 (2014-11-08)
------------------
* final and tested version of loader
* Latest Versions of charging launch machine tags
* Contributors: Jaime Pulido Fentanes

0.0.6 (2014-11-06)
------------------

0.0.5 (2014-10-23)
------------------
* Replacing opencv2 dependency with cv_bridge to be distribution independent.
  opencv2 does not exists under indogo anymore. cv_bridge is pulling in the correct opencv packages for both distributions.
* Contributors: Christian Dondrup

0.0.4 (2014-10-13)
------------------
* Merge branch 'hydro-devel' of https://github.com/strands-project/scitos_apps into hydro-devel
* Removed debug messages
* Undocking preemption bug solved.
* Contributors: Tom Krajnik

0.0.3 (2014-09-22)
------------------
* Hopefully the last missing dependencies...
* Contributors: Christian Dondrup

0.0.2 (2014-09-22)
------------------
* Added missing dependencies.
* Contributors: Christian Dondrup

0.0.1 (2014-09-22)
------------------
* [scitos_ptu] release preparations.
* [scitos_docking] Realease preparation
  * Moved header files to include
  * Created proper directories
  * Created install targets
  * Cleaned package.xml
  * Added license
* added opencv2
* added libncurses-dev as dependency
* [scitos_door_pass] Removed double dependencies and added move_base_msgs to package.xml
* Charging action and service is now part of the scitos_docking package.
* Moved action_buttons message to scitos_teleop package.
* Renamed ros_datacentre to mongodb_store
  This simply bulk replaces all ros_datacentre strings to mongodb_store strings inside files and also in file names.
  Needs `strands-project/ros_datacentre#76 <https://github.com/strands-project/ros_datacentre/issues/76>`_ to me merged first.
* adding machine tags to cmd_vel_mux, teleop_joystick and scitos docking launch files
* Merge pull request `#82 <https://github.com/strands-project/scitos_apps/issues/82>`_ from gestom/hydro-devel
  Charging now split into three action servers: docking, undocking and cha...
* Charging now split into three action servers: docking, undocking and chargingServer. ChargingServer can dock, undock, and calibrate depending on the command it receives. Docking and undocking servers receive MoveBase goal, so they can be integrated in Topological navigation.
* Using the `${catkin_EXPORTED_TARGETS}` macro in all the add_dependencies statements now.
* Changed bracketing to fix compilation error.
* Switched strands_datacentre to ros_datacentre in here
* Merge pull request `#50 <https://github.com/strands-project/scitos_apps/issues/50>`_ from hawesie/master
  Linking changes for OSX
* Merge branch 'master' of https://github.com/hawesie/scitos_apps
* Should fix the dependency problem for the data_centre
* added dependency for datacentre
* OS X linking
* Dependencies of CPtuClient
* Linking changes for OSX
* Docking position injection improvement.
* Minor issue with position injection timestamp.
* Attempt to resolve the undocking simulation failure.
* Merge pull request `#31 <https://github.com/strands-project/scitos_apps/issues/31>`_ from hawesie/master
  Seems to work OK.
* Merge branch 'master' of https://github.com/hawesie/scitos_apps into hawesie-master
* Position injection into AMCL after undocking.
* Dependecy bug corrected.
* Publishing after preemption suppressed.
* Charging in the dark
* Added MongoDB support to store calibration params. Corrected reports on success or failure. Command multiplexer made optional.
* Linking commands necessary for OS X.
* A potential bug, which caused the visual charging to fail in simulation, has been removed. Moreover, the undocking now starts with slowly opening the eyes. That gives the users some time to leave the area behind the robot.
* A potential bug, which caused the visual charging to fail in simulation, has been removed. Moreover, the undocking now starts with slowly opening the eyes. That gives the users some time to leave the area behind the robot.
* Added missing instruction.
* Updated documentation to cover actionlib.
* Added dependencies to CMakeLists, extended the testing mode of visual charging
* Test script modification.
* Test script modified.
* Calibration params now saved to /.charging.yaml. Code rewritted towards human redability. Docking/undocking can be initiated by joystick button X. Controller speed-up. Added test mode for automated trials and a test script.
* Docs update
* Documentation update.
  Removed redundant files.
* Added a client for testing, modified messages to report on progress and implemented simple self-diagnostic.
* Charging process now an actionlib server.
* Calibration parameters now saved in etc/charging.yaml, so calibration has to be performed only once.
* Minor changes to robot controllers.
* Solving compatibility issues with simulator.
* Explicit stop when waiting for charger signal.
* Rotation of the robot stopped after undocking and charging.
* Charging rejected on missing calibration parameters.
* No charging attempted without proper calibration params.
* Bug in retreival of camera parameters resolved.
* Stop command send explicitly after the docking or undocking terminates.
  Camera parameters can be changed on the fly.
* Small improvements to docking code.
* CMalelist improved to conform with source name change.
* Image processing now running on monochromatic image.
* Towards compatibility with the MORSE simulator:
  1) Listening to Battery state rather than Charging status.
  2) Image processing modified to include alpha channel processing.x
* Documentation added.
* Undocking + improvements to increase docking success rate.
* Added a robot station label.
  Updated a readme file.
* rosdep fixed for gsl
* Added gsl find and linking for compile on osx.
* fixed a renaming bug.
* Some urgent restructuring and adding of dependencies to make it build again.
* command now precedes timeout
* Solving compilation issues with scitos_messages.
* Cleanup.
* First verstion of the docking.
* Contributors: BFALacerda, Bruno Lacerda, Christian Dondrup, Computing, Jaime Pulido Fentanes, Marc Hanheide, Nick Hawes, Tom Krajnik, Tomas Krajnik, cdondrup, strands
