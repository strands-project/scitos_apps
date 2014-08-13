#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include "CDoorDetection.h"

ros::Subscriber scan_sub;

int measurements = 0;
float maxDistance = 3.0;	
int detections = 0;		
bool debug = true;
float doorWidth = 0;

typedef enum{
	IDLE,
	DETECT,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

EClimbState state = IDLE;
 
void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
	if (state == DETECT){
		SDoor door = detectDoor(scan_msg->ranges,scan_msg->angle_min,scan_msg->angle_increment,scan_msg->ranges.size(),maxDistance);
		if (door.found){
			doorWidth+=door.doorWidth;
			detections++;
			printf("Found\n");
		}
		else {
			printf("Not found\n");	
		}
		measurements--;
	}
}

void actionServerCallback()
{
	printf("Detecting\n");
	ros::NodeHandle n;
	scan_sub = n.subscribe("scan", 100, scanCallback);
	printf("Subscribed\n");
	state = DETECT;
	measurements = 30;
	detections = 0;
	doorWidth = 0;
	ros::Rate r(50); //hz
	while (state == DETECT && ros::ok()){
		if (measurements <= 0){
			// if (detections > 0) state = SUCCESS; else state = FAIL;
			// SUCCESS means that it ran to completion, result communicates open/closed
			state = SUCCESS;
		}
		ros::spinOnce();
		r.sleep();
	}
	bool open = false;
	if (detections > 0){
		float width=doorWidth/detections;
		open=true;
	}
	state = STOPPING;
	scan_sub.shutdown();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "doorDetection");
	ros::NodeHandle n;

	ros::Rate r(30); //hz
	
	while (ros::ok()){

		if(state == IDLE) {
			actionServerCallback();
		}
		
		if (state == STOPPING) {
			state = IDLE;
		}
		
		ros::spinOnce();
		r.sleep();
	}
}

