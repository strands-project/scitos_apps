#include <stdlib.h>
#include "scitos_docking/CDump.h"
#include "scitos_docking/CTimer.h"
#include "scitos_docking/CCircleDetect.h"
#include "scitos_docking/CTransformation.h"
#include "scitos_docking/CChargingClient.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <scitos_docking/ChargingAction.h>
#include <scitos_teleop/action_buttons.h>
#include <actionlib/server/simple_action_server.h>
#include <sensor_msgs/Joy.h>
#include <scitos_msgs/BatteryState.h>
#include <move_base_msgs/MoveBaseAction.h>
//#include <scitos_docking/Charging.h>
#include "scitos_docking/CChargingActions.h"


#define MAX_PATTERNS 10 

float ptuPan = 0.0;
bool success = false;
int failedToSpotStationCount=0;
scitos_docking::ChargingFeedback feedback;
scitos_docking::ChargingResult result;
char response[3000];
char posString[3000];
typedef actionlib::SimpleActionServer<scitos_docking::ChargingAction> Server;
typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> DockingServer;

int maxMeasurements = 100;
float dockingPrecision = 0.10;
float realPrecision,tangle,tdistance;

int stationSpotted = 0;
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
DockingServer *dockingServer;
DockingServer *undockingServer;

image_transport::Publisher imdebug;

int maxFailures=60;
TLogModule module = LOG_MODULE_MAIN;
int numSaved = 0;
CRawImage *image;
int waitCycles = 0;
int ptupos = 0;
CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;
CChargingClient chargingClient;
bool positionUpdate = false;

//position injection related
EState state = STATE_ROTATE;
EState lastState = STATE_IDLE;

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	if (positionUpdate){
		if (robot->poseSet == false){
			float phiDiff = robot->injectPhi- tf::getYaw(msg->orientation);
			while (phiDiff >= M_PI) phiDiff -= 2*M_PI;
			while (phiDiff < -M_PI) phiDiff += 2*M_PI;
			//ROS_INFO("A: %f %f %f\n",msg->position.x,msg->position.y,tf::getYaw(msg->orientation));
			if (fabs(robot->injectX-msg->position.x) < 0.01 && fabs(robot->injectY-msg->position.y) < 0.01 && fabs(phiDiff) < 0.01) robot->poseSet = true;
		}
	}
}

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
	failedToSpotStationCount = 0;
	image->getSaveNumber();
	state = STATE_IDLE;
	robot->progress = 10;
	robot->progressSpeed = 2.0;
	calibrated = trans->loadCalibration();
	stationSpotted = 0;
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
			if (waitCycles++ > 10){
				robot->lightsOff();
				success = true;
				state = STATE_TEST_SUCCESS;
			}
			break;
		case STATE_TEST2:
			if (robot->moveByDistance()){
				state = STATE_TEST3;
				robot->rotateByAngle(M_PI-0.2);
			}
			break;
		case STATE_TEST3:
			if (robot->rotateByAngle()){
				success = true;
				state = STATE_TEST_SUCCESS;
			}
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
			if (robot->rotateByAngle()){
				robot->halt();
				 state = STATE_UNDOCK_ADJUST_STEP1;
				 robot->lightsOn();
			}
			robot->controlHead(100,0,0);
			break;
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	STrackedObject own,station;
	if (state == STATE_INIT){
		robot->initCharging(chargerDetected,maxMeasurements);
		robot->movePtu(0,0);
		if (chargerDetected) state = STATE_WAIT;
		if (chargerDetected == false && fabs(ptuPan)<0.01) state = STATE_SEARCH; 
	}
	if (state == STATE_UNDOCK_INIT)
	{
		robot->movePtu(275,0);
		if (robot->wait()){
			robot->moveByDistance(-0.55);
			state = STATE_UNDOCK_MOVE;
			robot->controlHead(100,180,0);
			robot->moveHead();
		}
	}
	if ((int) state < (int)STATE_RETRY){
		if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
			delete image;
			ROS_DEBUG("Readjusting image format from %ix%i %ibpp, to %ix%i %ibpp.",image->width,image->height,image->bpp,msg->width,msg->height,msg->step/msg->width);
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
			stationSpotted++;
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
				case STATE_UNDOCK_ADJUST_STEP1:
					robot->adjust(station,station.y*1.1);
					robot->halt();
					state = STATE_UNDOCK_ADJUST_STEP2;
					break;
				case STATE_UNDOCK_ADJUST_STEP2:
					if (robot->adjust(station,0.0,0.05)){
						robot->halt();
						robot->measure(NULL,NULL,maxMeasurements);
						state = STATE_UNDOCK_MEASURE1;
					}
					break;
				case STATE_UNDOCK_MEASURE1:
					own = trans->getOwnPosition(objectArray);
					if (robot->measure(&own)){
						robot->halt();
						robot->measure(NULL,NULL,maxMeasurements);
						state = STATE_UNDOCK_MEASURE2;
					}
					break;				
				case STATE_UNDOCK_MEASURE2:
					own = trans->getOwnPosition(objectArray);
					if (robot->measure(&own)){
						robot->halt();
						robot->movePtu(0,0);
						sprintf(posString,"Robot position after undock: %f %f %f %f",own.x,own.y,own.z,own.yaw);
						printf("%s\n",response);
						success = true;
						robot->injectPosition(-own.z,-own.x,own.yaw);
						state = STATE_UNDOCKING_SUCCESS;
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
						if(trans->saveCalibration()){
							success = true;
							state = STATE_CALIBRATION_SUCCESS;
							sprintf(response,"Calibration OK.");
						} else{
							success = false;
							state = STATE_CALIBRATION_FAILURE;
							sprintf(response,"Calibration OK, but the calibration parameters were saved neither in a file nor in the STRANDS datacentre. Autonomous charging will work for now, but you will have to perform calibration again after restarting ROS or the robot.");
						}
					}
					break;
				case STATE_WAIT:
					own = trans->getOwnPosition(objectArray);
					if (robot->wait(&own,station,chargerDetected)){
						if (chargerDetected){
							 state = STATE_DOCKING_SUCCESS;
							 success = true;
						} else{
							state = STATE_RETRY;
						}
						realPrecision = sqrt(own.x*own.x+own.y*own.y);
					}
					break;
			}
		}else{
			failedToSpotStationCount++;
			if (state == STATE_SEARCH) failedToSpotStationCount=0;
		}
		if (failedToSpotStationCount > maxFailures){
			if (state == STATE_CALIBRATE){
				 state=STATE_CALIBRATION_FAILURE;
				 sprintf(response,"Cannot see the ROBOT STATION tag. Calibration failed.");
			} else if (state == STATE_UNDOCK_ADJUST_STEP1 || state == STATE_UNDOCK_MEASURE1 || state == STATE_UNDOCK_ADJUST_STEP2|| state == STATE_UNDOCK_MEASURE2){
				 sprintf(response,"Cannot see the ROBOT STATION tag. Undocking was not successfull.");
				 state=STATE_UNDOCKING_FAILURE;
			}else{
				 state=STATE_DOCKING_FAILURE; 
			}
		}
	}
}

void dockingServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, DockingServer* as)
{
	initComponents();
	timer.reset();
	timer.start();
	timeOut = 120000;
	move_base_msgs::MoveBaseResult result;
	if (calibrated==false){
		sprintf(response,"Cannot approach the charging station because the docking is not calibrated.\nRead the readme file on how to perform calibration procedure.");
		state = STATE_REJECTED;
	}else{
		state = STATE_INIT;
		robot->progress = 10;
		robot->lightsOn();
	}
	if (state == STATE_REJECTED){
		dockingServer->setAborted(result);
		robot->lightsOff();
		return;
	}
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT && state != STATE_PREEMPTED){
		usleep(200000);
	}
	if (state == STATE_PREEMPTED){
		dockingServer->setPreempted(result);
		state = STATE_IDLE;
		robot->movePtu(0,0);
		robot->lightsOff();
		return;
	}else if (state == STATE_ABORTED){
		dockingServer->setAborted(result);
		robot->movePtu(0,0);
		robot->lightsOff();
		return;
	}else if (success)
	{
		robot->progress = 100;
		dockingServer->setSucceeded(result);
		success = false;
	}else{
		dockingServer->setAborted(result);
	}
	robot->lightsOff();
}


void undockingServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, DockingServer* as)
{	
	initComponents();
	timer.reset();
	timer.start();
	timeOut = 120000;
	move_base_msgs::MoveBaseResult result;
	if (chargerDetected == false){
		state = STATE_REJECTED;
		sprintf(response,"Cannot undock because not on the charging station.");
	}else{
		robot->wait(100);
		state = STATE_UNDOCK_INIT;
	}
	if (state == STATE_REJECTED){
		undockingServer->setAborted(result);
		robot->lightsOff();
		return;
	}
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT && state != STATE_PREEMPTED){
		usleep(200000);
	}
	if (state == STATE_PREEMPTED){
		undockingServer->setPreempted(result);
		state = STATE_IDLE;
		robot->movePtu(0,0);
		robot->lightsOff();
		return;
	}else if (state == STATE_ABORTED){
		undockingServer->setAborted(result);
		robot->movePtu(0,0);
		robot->lightsOff();
		return;
	}else if (success)
	{
		robot->progress = 100;
		undockingServer->setSucceeded(result);
		success = false;
	}else{
		undockingServer->setAborted(result);
	}
	robot->lightsOff();
}

void actionServerCallback(const scitos_docking::ChargingGoalConstPtr& goal, Server* as)
{
	initComponents();
	timer.reset();
	timer.start();
	timeOut = (int) goal->Timeout*1000;
	if (goal->Command == "charge"){
		if (calibrated==false){
			sprintf(response,"Cannot approach the charging station because the docking is not calibrated.\nRead the readme file on how to perform calibration procedure.");
			state = STATE_REJECTED;
		}else{
			state = STATE_INIT;
			robot->progress = 10;
			robot->lightsOn();
		}
	}
	if (goal->Command == "test"){
		 robot->lightsOn();
		 ptupos = (int)goal->Timeout;
		 robot->movePtu(275,0);
		 waitCycles = 0;
		 state = STATE_TEST1;
	}
	if (goal->Command == "calibrate"){
		state = STATE_CALIBRATE;
		robot->measure(NULL,NULL,maxMeasurements);
		robot->lightsOn();
	}
	if (goal->Command == "undock"){
		if (chargerDetected == false){
			state = STATE_REJECTED;
			sprintf(response,"Cannot undock because not on the charging station.");
		}else{
			robot->wait(100);
			state = STATE_UNDOCK_INIT;
		}
	}
	if (state == STATE_REJECTED){
		result.Message = response;
		server->setAborted(result);
		robot->lightsOff();
		return;
	}
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT && state != STATE_PREEMPTED){
		usleep(200000);
		server->publishFeedback(feedback);
	}
	result.Message = response;
	if (state == STATE_PREEMPTED){
		server->setPreempted(result);
		state = STATE_IDLE;
		robot->lightsOff();
		return;
	}else if (state == STATE_ABORTED){
		server->setAborted(result);
		robot->lightsOff();
		return;
	}else if (success)
	{
		robot->progress = 100;
		server->setSucceeded(result);
		success = false;
	}else{
		server->setAborted(result);
	}
	robot->lightsOff();
}

void ptuCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	for (int i = 0;i<2;i++){
		if (msg->name[i] == "pan") ptuPan = msg->position[i];
	}
}

void joyCallback(const scitos_teleop::action_buttons::ConstPtr &msg)
{
	if (msg->X && state == STATE_IDLE && server->isActive() == false && undockingServer->isActive() == false && dockingServer->isActive()==false){
		if (chargerDetected){
			robot->wait(100);
			state = STATE_UNDOCK_INIT;
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
	if (state == STATE_DOCKING_FAILURE) sprintf(response,"The robot has failed reach the charger, which has been seen %i times.",stationSpotted);
	if (state == STATE_UNDOCKING_SUCCESS) sprintf(response,"Undocking  successfully completed.%s",posString);
	if (state == STATE_TEST_SUCCESS) sprintf(response,"Testrun successfully completed at %f %f.",cos(tangle)*tdistance+0.55,sin(tangle)*tdistance);
	if (state == STATE_UNDOCKING_FAILURE) sprintf(response,"Undocking  not completed.");
	if (state == STATE_CALIBRATION_SUCCESS) sprintf(response,"Calibration OK.");
//	if (state == STATE_CALIBRATION_FAILURE) sprintf(response,"Calibration failed.");
	if (state == STATE_TIMEOUT) sprintf(response,"Requested action was not completed in the requested time. Station spotted %i times.\n",stationSpotted);
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
		if (positionUpdate && robot->poseSet == false) robot->injectPosition(); 
		usleep(30000);
		if (state!=lastState){
			sprintf(status,"Charging service is %s",stateStr[state]);
			feedback.Progress = (int)robot->progress;
			feedback.Message = stateStr[state];
			if (server->isActive()) server->publishFeedback(feedback);
		}
		if (server->isActive()||undockingServer->isActive()||dockingServer->isActive()){
			feedback.Message = stateStr[state];
			feedback.Progress = (int)robot->progress;
			if (robot->actionStuck()) feedback.Level = 1; else feedback.Level = 0;
		}
		if (((server->isPreemptRequested() && server->isActive())  || (undockingServer->isActive() && undockingServer->isPreemptRequested())|| (dockingServer->isActive() && dockingServer->isPreemptRequested()))&& state != STATE_IDLE){
			state = STATE_PREEMPTED;
			robot->halt();
			ros::spinOnce();
			if (server->isActive()||undockingServer->isActive()||dockingServer->isActive()) result.Message = "Current action preempted by external request.";
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
	success = false;
	image_transport::Subscriber subim = it.subscribe("head_xtion/rgb/image_mono", 1, imageCallback);
	nh->param("positionUpdate",positionUpdate,false);
        imdebug = it.advertise("/charging/processedimage", 1);
	ros::Subscriber subodo = nh->subscribe("odom", 1, odomCallback);
	ros::Subscriber subcharger = nh->subscribe("battery_state", 1, batteryCallBack);
	ros::Subscriber subcamera = nh->subscribe("head_xtion/rgb/camera_info", 1,cameraInfoCallBack);
	ros::Subscriber joy_sub_ = nh->subscribe("/teleop_joystick/action_buttons", 10, joyCallback);
	ros::Subscriber ptu_sub_ = nh->subscribe("/ptu/state", 10, ptuCallback);
	ros::Subscriber robot_pose = nh->subscribe("/robot_pose", 1000, poseCallback);
	server = new Server(*nh, "chargingServer", boost::bind(&actionServerCallback, _1, server), false);
	dockingServer = new DockingServer(*nh, "docking", boost::bind(&dockingServerCallback, _1, dockingServer), false);
	undockingServer = new DockingServer(*nh, "undocking", boost::bind(&undockingServerCallback, _1, undockingServer), false);

	server->start();
	dockingServer->start();
	undockingServer->start();
	ROS_DEBUG("Server running");
	while (ros::ok()) mainLoop();
	delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}
