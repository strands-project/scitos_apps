#include <stdlib.h>
#include "ros/ros.h"
#include <scitos_msgs/EnableMotors.h>
#include <scitos_msgs/MotorStatus.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <scitos_virtual_bumper/virtualbumperevent.h>
#include <scitos_virtual_bumper/virtualbumperreport.h>

static int maxSteady = 30;
ros::Subscriber odomSub,cmdSub,motSub,poseSub;
ros::Publisher bumperEvent,bumperReport;
ros::ServiceClient enableMotors; 
bool steady = false;
bool stopped = true;
int steadyCnt = 0;

bool freeRunByVirtualBumper = false;
bool stopRunByVirtualBumper = false;

bool freeRun = false;
geometry_msgs::PoseStamped currentPose;
geometry_msgs::PoseStamped initialPosition;
geometry_msgs::PoseStamped finalPosition;

double translationThreshold = 0.2;
double rotationThreshold = 0.1;

void reportPosition(bool pushed)
{
	scitos_virtual_bumper::virtualbumperevent event;
	event.pose = currentPose;
	event.freeRunStarted = pushed;
	event.freeRunByVirtualBumper = freeRunByVirtualBumper;
	event.stopRunByVirtualBumper = stopRunByVirtualBumper;
	if (pushed) initialPosition = currentPose; else finalPosition = currentPose;
	bumperEvent.publish(event);
}

void motorCallback(const scitos_msgs::MotorStatus &msg)
{	
	bool freeRunMsg = (msg.free_run!=0);
	if (freeRun != freeRunMsg) 
	{
		reportPosition(freeRunMsg);

		/*is freerun over and was is initiateb by the virtual bumper?*/
		if (freeRunMsg ==false && freeRunByVirtualBumper)
		{
			float dx = (finalPosition.pose.position.x - initialPosition.pose.position.x); 
			float dy = (finalPosition.pose.position.y - initialPosition.pose.position.y);
			float distance = sqrt(dx*dx+dy*dy);
			float angle = 2*fabs(atan2(initialPosition.pose.orientation.w,initialPosition.pose.orientation.z)-atan2(finalPosition.pose.orientation.w,finalPosition.pose.orientation.z));
			if (angle > M_PI) angle = fabs(angle - 2*M_PI);
			if (angle > rotationThreshold || distance > translationThreshold){	
				scitos_virtual_bumper::virtualbumperreport report;
				report.initial = initialPosition;
				report.final   = finalPosition;
				report.distance = distance;
				report.angle = angle;
				report.stopRunByVirtualBumper = stopRunByVirtualBumper;
				bumperReport.publish(report);
			}
			freeRunByVirtualBumper = stopRunByVirtualBumper = false;
		}
	}
	freeRun = freeRunMsg;
}

void cmdCallback(const geometry_msgs::Twist &msg)
{
	if (msg.linear.x == 0 && msg.angular.z == 0) stopped = true; else stopped = false;
}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	currentPose.header.stamp = ros::Time::now();
	currentPose.header.frame_id = "/map";
	currentPose.pose.position = msg->position;
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
		freeRunByVirtualBumper = true;
	}

	/*am I too long on free run?*/
	if (steadyCnt > maxSteady && steady && freeRun && freeRunByVirtualBumper)
	{
		scitos_msgs::EnableMotors motors;
		motors.request.enable = true;
		enableMotors.call(motors);
		stopRunByVirtualBumper = true;
	}
	if (stopped && steady) steadyCnt++; else steadyCnt = 0;
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "virtual_bumper");
	ros::NodeHandle *nh = new ros::NodeHandle;

	nh->param("reportTranslationThreshold",translationThreshold,0.2);
	nh->param("reportRotationThreshold",rotationThreshold,0.1);
	enableMotors = nh->serviceClient<scitos_msgs::EnableMotors>("enable_motors");
	odomSub = nh->subscribe("odom", 1, odomCallback);
	cmdSub = nh->subscribe("cmd_vel", 1, cmdCallback);
	motSub = nh->subscribe("motor_status", 1, motorCallback);
	poseSub = nh->subscribe("robot_pose", 1, poseCallback);
        bumperEvent = nh->advertise<scitos_virtual_bumper::virtualbumperevent>("/virtual_bumper_event", 1);
        bumperReport = nh->advertise<scitos_virtual_bumper::virtualbumperreport>("/virtual_bumper_report", 1);

	ros::spin();
	return 0;
}
