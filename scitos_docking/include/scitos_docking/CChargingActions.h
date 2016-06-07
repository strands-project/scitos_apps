#ifndef CCHARGINGACTIONS_H
#define CCHARGINGACTIONS_H

/**
@author Tom Krajnik
*/
#include "scitos_docking/CChargingState.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <sys/time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "scitos_docking/CTransformation.h"
#include "scitos_docking/CTimer.h"
#include "scitos_docking/CLightClient.h"
#include "scitos_docking/CHead.h"
#include <tf/tf.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#define TIMEOUT_INTERVAL 40000

class CChargingActions
{
	public:
		CChargingActions(ros::NodeHandle *n);
		~CChargingActions();
		void moveHead();
		void movePtu(int pan,int tilt);
		void controlHead(int lids,int tilt, int pan);
		bool rotateByAngle(float angle = .0);
		bool moveByDistance(float distance = .0);
		void setObstacleDistance(float value);
		bool search();
		bool measure(STrackedObject *o1,STrackedObject *o2=NULL,int count = 0,bool ml=true);
		bool wait(int count = 0);
		void initCharging(bool isCharging,int maxMeasurements);
		bool approach(STrackedObject station,float dist = 0.0);
		bool adjust(STrackedObject station,float in = 0.0,float tol = 0.05);
		bool dock(STrackedObject station);
		bool wait(STrackedObject *own,STrackedObject station,bool chargerDetect,bool terminate);
		bool halt();
		bool actionStuck();
		bool testMove();
		void updatePosition(const nav_msgs::Odometry &msg);
		float progress,progressSpeed,lastProgress, startProg;
		bool headOn(int value = 0);
		bool headOff(int value = 0);
		void lightsOn();
		void lightsOff();
		void injectPosition(float x,float y,float phi); 
		void injectPosition(); 
		bool updateInjectionPositions(int stationID);

		float injectX,injectY,injectPhi;
		bool poseSet;
		float ptuPan;
	private:
		float obstacleDistance;
		CTimer timer;
		ros::NodeHandle *nh;
		ros::Publisher cmd_vel;
		ros::Publisher cmd_head;
		ros::Publisher cmd_ptu;
		geometry_msgs::Twist base_cmd;
		sensor_msgs::JointState head;
		sensor_msgs::JointState ptu;
		nav_msgs::Odometry current;
		float currentAngle,lastAngle;
		float warningLevel;
		CLightClient light;
		CHead headCtrl;
		ros::Publisher poseInjection;
		std::string frame;
		double dockPositionX;
		double dockPositionY;
		double dockPositionPhi;
		ros::Time injectionTime;
};

#endif
