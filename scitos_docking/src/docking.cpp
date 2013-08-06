#include <stdlib.h>
#include "CDump.h"
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include "CChargingClient.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <scitos_apps_msgs/ChargingAction.h>
#include <scitos_apps_msgs/action_buttons.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/Joy.h>
#include <scitos_msgs/BatteryState.h>
#include <scitos_apps_msgs/Charging.h>
#include "CChargingActions.h" 

#define MAX_PATTERNS 10 
	
scitos_apps_msgs::ChargingFeedback feedback;
scitos_apps_msgs::ChargingResult result;
char response[1000];
typedef actionlib::SimpleActionServer<scitos_apps_msgs::ChargingAction> Server;

int maxMeasurements = 100;
float dockingPrecision = 0.1;
float realPrecision,tangle,tdistance;

bool calibrated = true;
CTimer timer;
int timeOut = 120;
int  defaultImageWidth= 320;
int  defaultImageHeight = 240;
float circleDiameter = 0.05;
float rotateBy = 0;
bool chargerDetected = false;
CChargingActions *robot;

ros::NodeHandle *nh;
Server *server;

image_transport::Publisher imdebug;

int maxFailures=60;
TLogModule module = LOG_MODULE_MAIN;
int numSaved = 0;
CRawImage *image;

CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;
CChargingClient chargingClient;

EState state = STATE_ROTATE;
EState lastState = STATE_IDLE;

void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg)
{
	trans->updateParams(msg.K[2],msg.K[5],msg.K[0],msg.K[4]);
}

void batteryCallBack(const scitos_msgs::BatteryState &msg)
{
	chargerDetected = msg.charging;
}

int initComponents()
{
	image->getSaveNumber();
	state = STATE_IDLE;
	robot->progress = 10;
	robot->progressSpeed = 2.0;
	calibrated = trans->loadCalibration();
}

void odomCallback(const nav_msgs::Odometry &msg)
{
	robot->updatePosition(msg);
	switch (state){
		case STATE_SEARCH:
			robot->search();
			if (robot->progress > 95){
				state = STATE_ABORTED;
				sprintf(response,"No charging station detected."); 
			}
			break;
		case STATE_TEST1:
			if (robot->rotateByAngle())state = STATE_TEST2;
			break;
		case STATE_TEST2:
			if (robot->moveByDistance()){
				 state = STATE_TEST3;
				 robot->rotateByAngle(M_PI-0.2);
			}
			break;
		case STATE_TEST3:
			if (robot->rotateByAngle())state = STATE_TEST_SUCCESS;
			break;
		case STATE_ROTATE:
			if (robot->rotateByAngle()) state = STATE_MOVE_TO;
			robot->controlHead(100,-rotateBy/M_PI*180,0);
			break;
		case STATE_ROTATE_BACK:
			if (robot->rotateByAngle()) state = STATE_ADJUST;
			robot->controlHead(100,0,0);
			break;
		case STATE_MOVE_TO:
			if (robot->moveByDistance()){
				state = STATE_ROTATE_BACK; 
				if (rotateBy > 0) rotateBy = -M_PI/2; else rotateBy = M_PI/2;
				robot->rotateByAngle(rotateBy);
			}
			break;
		case STATE_RETRY:
			if (robot->moveByDistance()) state = STATE_APPROACH;
			robot->controlHead(100,0,0);
			break;
		case STATE_UNDOCK_MOVE: 
			robot->controlHead(100,180,0);
			if (robot->moveByDistance()){
				rotateBy = (M_PI+0.01);
				robot->rotateByAngle(rotateBy);
				state = STATE_UNDOCK_ROTATE;
			}
			break;
		case STATE_UNDOCK_ROTATE:
			if (robot->rotateByAngle()) state = STATE_UNDOCKING_SUCCESS;
			robot->controlHead(100,0,0);
			break;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	static int failedToSpotStationCount;
	STrackedObject own,station;
	if (state == STATE_INIT){
		robot->initCharging(chargerDetected,maxMeasurements);
		if (chargerDetected) state = STATE_WAIT; else state = STATE_SEARCH; 
	}
	if ((int) state < (int)STATE_RETRY){
		if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
			delete image;
			ROS_INFO("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",image->width,image->height,image->bpp,msg->width,msg->height,msg->step/msg->width);
			image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
		}
		memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);

		//search image for circles
		for (int i = 0;i<3;i++){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
			objectArray[i].valid = false;
			if (currentSegmentArray[i].valid)objectArray[i] = trans->transform(currentSegmentArray[i]);
		}
		//and publish the result
		memcpy((void*)&msg->data[0],image->data,msg->step*msg->height);
		imdebug.publish(msg);

		//is the ROBOT STATION label visible ?	
		station = trans->getDock(objectArray);
		if (station.valid){
			failedToSpotStationCount = 0;
			robot->controlHead(100,180/M_PI*atan2(station.y,station.x),-180/M_PI*atan2(station.z-0.2,station.x));
			switch (state){
				case STATE_SEARCH:
					state = STATE_APPROACH;
					if (robot->approach(station,station.x)) state = STATE_ADJUST;
					break;
				case STATE_APPROACH:
					if (robot->approach(station)){
						state = STATE_ADJUST;
						robot->adjust(station,station.y);
					}
					break;
				case STATE_ADJUST:
					if (robot->adjust(station)){
						state = STATE_MEASURE;
						robot->measure(NULL,NULL,maxMeasurements);
					}
					break;
				case STATE_DOCK:
					if (robot->dock(station)){
						state = STATE_WAIT;
						robot->measure(NULL,NULL,4*maxMeasurements,false);
					}
					break;
				case STATE_MEASURE:
					own = trans->getOwnPosition(objectArray);
					if (robot->measure(&own)){
						robot->halt();
						if (fabs(own.x) < dockingPrecision){
							state = STATE_DOCK;
							robot->startProg = station.x;
						} else{
							state = STATE_ROTATE;
							rotateBy = M_PI/2;
							if (own.x > 0) rotateBy = -rotateBy;
							rotateBy += sin((own.x+trans->ownOffset.x)/(own.z+trans->ownOffset.z));
							robot->rotateByAngle(rotateBy);
							robot->moveByDistance(fabs(own.x));
						}
					}
					break;
				case STATE_CALIBRATE:
					trans->clearOffsets();
					own = trans->getOwnPosition(objectArray);
					if (robot->measure(&own,&station)){
						trans->updateCalibration(own,station);
						if(trans->saveCalibration()) state = STATE_CALIBRATION_SUCCESS; else state = STATE_CALIBRATION_FAILURE;
					}
					break;
				case STATE_WAIT:
					own = trans->getOwnPosition(objectArray);
					if (robot->wait(&own,station,chargerDetected)){
						if (chargerDetected) state = STATE_DOCKING_SUCCESS; else state = STATE_RETRY;
						realPrecision = sqrt(own.x*own.x+own.y*own.y);
					}
					break;
			}
		}else{
			failedToSpotStationCount++;
			if (state == STATE_SEARCH) failedToSpotStationCount=0;
		}
		if (failedToSpotStationCount > maxFailures) state=STATE_DOCKING_FAILURE; 
	}
}

void actionServerCallback(const scitos_apps_msgs::ChargingGoalConstPtr& goal, Server* as)
{
	initComponents();
	if (goal->Command == "charge"){
		if (calibrated==false){
			sprintf(response,"Cannot approach the charging station because the docking is not calibrated.\nRead the readme file on how to perform calibration procedure.");
			state = STATE_REJECTED;
		}else{
			state = STATE_INIT;
			robot->progress = 10;
		}
	}
	if (goal->Command == "test"){
		 tangle = (rand()%100)/100.0;
		 tdistance = 2.0*(rand()%100)/100;
		 robot->rotateByAngle(tangle); 
		 robot->moveByDistance(tdistance);
		 ROS_INFO("Testing %f %f",tangle,tdistance);
		 state = STATE_TEST1;
	}
	if (goal->Command == "calibrate"){
		state = STATE_CALIBRATE;
		robot->measure(NULL,NULL,maxMeasurements);
	}
	if (goal->Command == "undock"){
		if (chargerDetected == false){
			state = STATE_REJECTED;
			sprintf(response,"Cannot undock because not on the charging station.");
		}else{
			robot->moveByDistance(-0.55);
			state = STATE_UNDOCK_MOVE;
			robot->controlHead(100,180,0);
			robot->moveHead();
		}
	}
	if (state == STATE_REJECTED){
		result.Message = response;
		server->setAborted(result);
		return;
	}
	timer.reset();
	timer.start();
	timeOut = (int) goal->Timeout*1000;
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT){
		usleep(200000);
		server->publishFeedback(feedback);
		ROS_INFO("ROBOT %s Progress: %.0f %.2f Time: %i/%i",stateStr[state],robot->progress,robot->progressSpeed,timer.getTime(),timeOut);
	}
	result.Message = response;
	if (state == STATE_ABORTED){
		server->setAborted(result);
		return;
	}	
	robot->progress = 100;
	server->setSucceeded(result);
}

void joyCallback(const scitos_apps_msgs::action_buttons::ConstPtr &msg)
{
	if (msg->X && state == STATE_IDLE && server->isActive() == false){
		if (chargerDetected){
			state = STATE_UNDOCK_MOVE;
			chargingClient.initiateUndocking();
		} else {
			state = STATE_INIT;
			chargingClient.initiateCharging();
		}
	}
}

void state_cleanup()
{
	if (state == STATE_DOCKING_SUCCESS) sprintf(response,"The robot has successfully reached the charger in %i s with precision %.0f mm.",timer.getTime()/1000,realPrecision*1000);
	if (state == STATE_DOCKING_FAILURE) sprintf(response,"The robot has failed reached the charger.");
	if (state == STATE_UNDOCKING_SUCCESS) sprintf(response,"Undocking  successfully completed.");
	if (state == STATE_TEST_SUCCESS) sprintf(response,"Testrun successfully completed at %f %f.",cos(tangle)*tdistance+0.55,sin(tangle)*tdistance);
	if (state == STATE_UNDOCKING_FAILURE) sprintf(response,"Undocking  not completed.");
	if (state == STATE_CALIBRATION_SUCCESS) sprintf(response,"Calibration OK.");
	if (state == STATE_CALIBRATION_FAILURE) sprintf(response,"Calibration failed.");
	if (state == STATE_TIMEOUT) sprintf(response,"Requested action was not completed in the requested time.");
	if ((int) state >= STATE_DOCKING_SUCCESS && (int)state <= STATE_TIMEOUT){
		robot->progress = 100;
		state = STATE_IDLE;
		robot->moveHead();
		robot->halt();
		ros::spinOnce();
	}
}



void mainLoop()
{
	char status[1000];
	int a = 0;
	while (ros::ok()){
		if (timeOut < timer.getTime() && state != STATE_IDLE ) state = STATE_TIMEOUT;
		if (state != STATE_IDLE) robot->moveHead();
		ros::spinOnce();
		usleep(30000);
		if (state!=lastState){
			sprintf(status,"Charging service is %s",stateStr[state]);
			feedback.Progress = (int)robot->progress;
			feedback.Message = stateStr[state];
			server->publishFeedback(feedback);
		}
		if (server->isActive()){
			feedback.Message = stateStr[state];
			feedback.Progress = (int)robot->progress;
			if (robot->actionStuck()) feedback.Level = 1; else feedback.Level = 0;
		}
		if (server->isPreemptRequested()){
			state = STATE_ABORTED;
			robot->halt();
			ros::spinOnce();
			if (server->isActive()==false){
				result.Message = "Current action preempted by external request.";
				state = STATE_IDLE;
				server->setPreempted(result);
			}
		}
		state_cleanup();
		lastState = state;
	}
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "charging");
	nh = new ros::NodeHandle;
	robot = new CChargingActions(nh);
	image_transport::ImageTransport it(*nh);

	dump = new CDump(NULL,256,1000000);
	image = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	trans = new CTransformation(circleDiameter,nh);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	initComponents();

	image_transport::Subscriber subim = it.subscribe("head_xtion/rgb/image_mono", 1, imageCallback);
        imdebug = it.advertise("/charging/processedimage", 1);
	ros::Subscriber subodo = nh->subscribe("odom", 1, odomCallback);
	ros::Subscriber subcharger = nh->subscribe("battery_state", 1, batteryCallBack);
	ros::Subscriber subcamera = nh->subscribe("head_xtion/rgb/camera_info", 1,cameraInfoCallBack);
	ros::Subscriber joy_sub_ = nh->subscribe("/teleop_joystick/action_buttons", 10, joyCallback);
	server = new Server(*nh, "chargingServer", boost::bind(&actionServerCallback, _1, server), false);

	server->start();
	ROS_INFO("Server running");
	while (ros::ok()) mainLoop();
	delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}
