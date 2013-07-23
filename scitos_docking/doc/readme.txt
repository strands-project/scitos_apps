The docking service detects and guides the robot to its charging station.
The service is based on detection of the 'o' letters of the 'ROBOT STATION' tag in the robot camera image.
Prior to use, the method has to establish the docking station reference frame relative too the 


To setup:

Print the station tag on a A4 paper. Do not let the printer to resize it.
Center the robot at the charging station.
Display the robot camera image and fix the tag on the wall approximattelly in the image center.
Tell the system to measure its position relatively to the charger by calling 'rosservice call /chargingSrv calibrate 100'.

To use:

Just point the 
