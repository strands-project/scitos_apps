#include <stdlib.h>
#include "ros/ros.h"
#include <scitos_msgs/EnableMotors.h>
#include <scitos_msgs/MotorStatus.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>

static int maxSteady = 30;
ros::Subscriber odomSub,cmdSub,motSub;
ros::ServiceClient enableMotors; 
bool steady = false;
bool stopped = true;
int steadyCnt = 0;

bool freeRun = false;

void motorCallback(const scitos_msgs::MotorStatus &msg)
{
	freeRun = msg.free_run;
}

void cmdCallback(const geometry_msgs::Twist &msg)
{
	if (msg.linear.x == 0 && msg.angular.z == 0) stopped = true; else stopped = false;
}

void odomCallback(const nav_msgs::Odometry &msg)
{
	if (msg.twist.twist.linear.x == 0 && msg.twist.twist.angular.z == 0) steady=true; else steady=false;
	/*standing long enough with motors on and suddenly wheels turned?*/
	if (steadyCnt > maxSteady && steady == false && freeRun == false)
	{
		scitos_msgs::EnableMotors motors;
		motors.request.enable = false;
		enableMotors.call(motors);
	}

	/*am I too long on free run?*/
	if (steadyCnt > maxSteady && steady && freeRun)
	{
		scitos_msgs::EnableMotors motors;
		motors.request.enable = true;
		enableMotors.call(motors);
	}
	if (stopped && steady) steadyCnt++; else steadyCnt = 0;
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "virtual_bumper");
	ros::NodeHandle *nh = new ros::NodeHandle;

	enableMotors = nh->serviceClient<scitos_msgs::EnableMotors>("enable_motors");
	odomSub = nh->subscribe("odom", 1, odomCallback);
	cmdSub = nh->subscribe("cmd_vel", 1, cmdCallback);
	motSub = nh->subscribe("motor_status", 1, motorCallback);

	ros::spin();
	return 0;
}
