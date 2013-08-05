#ifndef CCHARGINGACTIONS_H
#define CCHARGINGACTIONS_H

/**
@author Tom Krajnik
*/
#include "CChargingState.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sys/time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "CTransformation.h"
#include "CTimer.h"
#include <tf/tf.h>

#define TIMEOUT_INTERVAL 40000

class CChargingActions
{
	public:
		CChargingActions(ros::NodeHandle *n);
		~CChargingActions();
		void moveHead();
		void controlHead(int lids,int tilt, int pan);
		bool rotateByAngle(float angle = .0);
		bool moveByDistance(float distance = .0);
		bool search();
		bool measure(STrackedObject *o1,STrackedObject *o2=NULL,int count = 0,bool ml=true);
		void initCharging(bool isCharging,int maxMeasurements);
		bool approach(STrackedObject station,float dist = 0.0);
		bool adjust(STrackedObject station,float in = 0.0);
		bool dock(STrackedObject station);
		bool wait(STrackedObject own,STrackedObject station,bool chargerDetect);
		bool halt();
		bool actionStuck();
		bool testMove();
		void updatePosition(const nav_msgs::Odometry &msg);
		float progress,progressSpeed,lastProgress, startProg;
	private:
		CTimer timer;
		ros::NodeHandle *nh;
		ros::Publisher cmd_vel;
		ros::Publisher cmd_head;
		geometry_msgs::Twist base_cmd;
		sensor_msgs::JointState head;
		nav_msgs::Odometry current;
		float currentAngle,lastAngle;
		float warningLevel;
};

#endif
