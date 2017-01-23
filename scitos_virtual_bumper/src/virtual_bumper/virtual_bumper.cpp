#include <stdlib.h>
#include "ros/ros.h"
#include <scitos_msgs/EnableMotors.h>
#include <scitos_msgs/MotorStatus.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

static int maxSteady = 30;
ros::Subscriber odomSub,cmdSub,motSub,poseSub;
geometry_msgs::PoseStamped currentPose;
ros::Publisher bumperPose;
ros::ServiceClient enableMotors; 
bool steady = false;
bool stopped = true;
int steadyCnt = 0;

bool freeRun = false;

void reportPosition(bool pushed)
{
	geometry_msgs::PoseStamped pos;
	pos.header.frame_id = "/map";
	pos.header.stamp = ros::Time::now();
	pos.pose.position.x = currentPose.pose.position.x;
	pos.pose.position.y = currentPose.pose.position.y;
	if (pushed == false) pos.pose.position.z = -1;
	if (pushed == true) pos.pose.position.z = -1;
	pos.pose.orientation = currentPose.pose.orientation; 
	bumperPose.publish(pos);
}

void motorCallback(const scitos_msgs::MotorStatus &msg)
{
	freeRun = msg.free_run;
}

void cmdCallback(const geometry_msgs::Twist &msg)
{
	if (msg.linear.x == 0 && msg.angular.z == 0) stopped = true; else stopped = false;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	currentPose.pose.position.x = msg->position.x;
	currentPose.pose.position.y = msg->position.y;
	currentPose.pose.orientation = msg->orientation;
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
	poseSub = nh->subscribe("robot_pose", 1, poseCallback);
        bumperPose = nh->advertise<geometry_msgs::PoseStamped>("/virtual_bumper", 1);

	ros::spin();
	return 0;
}
