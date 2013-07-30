The docking service detects and guides the robot to its charging station.
The service is based on detection of the 'o' letters of the 'ROBOT STATION' tag in the robot camera image.
Prior to use, the method has to establish the docking station reference frame relative to the robot position.

To setup:

Print the station tag on a A4 paper. Do not let the printer to resize it.
Center the robot at the charging station.
Display the robot camera image and fix the tag on the wall approximattelly in the image center.
Tell the system to measure its position relatively to the charger by calling a service /chargingSrv calibrate 100.
The robot should move its eyelids to indicate progress of the calibration process.

To use:

Just point the robot aproximatelly in the direction of the charging station and call the service /chargingSrv charge n, where n is the timeout in seconds.
To leave the charging station, call /chargingSrv undock n. As before, n is the timeout.
