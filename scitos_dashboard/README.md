scitos_dashboard
================

This package provides an rqt_robot_dashboard for the Scitos robot. The dashboard displays motor and battery status, and allows the motors to be stopped or free-run enabled.

Installation
------------
This should not require any additional packages to be installed...

Running
-------

On your off-robot pc:
```
export ROS_MASTER_URI=http://bob:11311   # or not-bob
rosrun scitos_dashboard scitos_dashboard
```

This brings up a small dashboard window, which is an rqt docking pane:

![ScreenShot](https://raw.github.com/cburbridge/scitos_apps/master/scitos_dashboard/doc/dash.png)

From left to right, the widgets are:
* Diagnostics
* ROS Console
* Motor Status
* Battery Status
* Robot Mileage in metres

