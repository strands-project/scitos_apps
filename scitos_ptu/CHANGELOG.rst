^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package scitos_ptu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* [scitos_ptu] release preparations.
* Merge branch 'hydro-devel' into dependencies
  Conflicts:
  scitos_door_pass/CMakeLists.txt
  scitos_door_pass/package.xml
  scitos_teleop/CMakeLists.txt
  scitos_teleop/package.xml
* Fixing the maintainer and author entries in package xml files that I am involved in.
* [scitos_ptu] adding actionlib to CMakeLists
* [scitos_ptu] Prerelease cleanup
  Removed unused toy node.
* Increased the time out time from 3 to 10 seconds
* adding respawning launch for metric maps ptu action server
* Bugfix in ptu_action_server_metric_map. Resetting the aborted flag
* accepting preemption during initial pose
* metric map action server now preempts flir action server instead of waiting for it to finish
* Bugfix flir_ptu_action_server
* Preempting the underlying action server
* Added preemption method to flir_ptu_action_server
* Metric map sweep publishes log string when preempted
* fixed typos
* Metric map now preemptable
* new node name
* default joint names for when in simulation
* correcting action message import
* ptu launch file
* adding flir_ptu_d46 action server messages
* moving scripts up
* Renamed the action server
* renamed the action server
* Added a log topic for the ptu server
* new ptu server that logs to the database
* action server feedback message
* Added feedback in the ptu action server
* Action message with fields for various parameters
* Moving the action file to the root of the package
* updated the PTU action server to check that the desired position has been reacheed using the /ptu/state message
* package dependencies
* action server for ptu
* Restructuring and renaming.
* Contributors: Chris Burbridge, Christian Dondrup, Rares Ambrus, annotator, cburbridge, cdondrup, default, rares
