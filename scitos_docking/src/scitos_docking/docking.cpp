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
#include "scitos_docking/CChargingActions.h"


#define MAX_PATTERNS 10 

float ptuPan = 0.0;
float ptuTilt = -15.0;
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
int timeOut = 120000;
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

int onBattery = 0;
int maxFailures=150;
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
bool headRestart = false;

//position injection related
EState state = STATE_ROTATE;
EState lastState = STATE_IDLE;

// Subscribers

image_transport::Subscriber subim; 
image_transport::Subscriber subdepth;
ros::Subscriber subodo;
ros::Subscriber subcharger;
ros::Subscriber subcamera;
ros::Subscriber joy_sub_;
ros::Subscriber ptu_sub_;
ros::Subscriber robot_pose;


void subscribe();
void unsubscribe();


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
	if (state ==  STATE_HEAD_OFF){
		if (robot->headOff()) state = STATE_DOCKING_SUCCESS;
	}
	if (state ==  STATE_HEAD_ON){
		if (robot->headOn()){
			robot->wait(100);
			state = STATE_UNDOCK_INIT;
		}
	}

/*	if (onBattery == 1000 && state == STATE_IDLE && headRestart == false)
	{
		state = STATE_HEAD_RESTART;
		headRestart = true;
		robot->headSwitch(false);
	}
	if (headRestart == true)
	{
		if (onBattery == 1200) robot->headSwitch(true);
		if (onBattery == 1500){
			headRestart = false;
			if (state == STATE_HEAD_RESTART) state = STATE_IDLE;
		}
		onBattery++;
	}
	if (state == STATE_IDLE)
	{
		if (chargerDetected) onBattery++; else onBattery=0;
	}*/
}

int initComponents()
{
	failedToSpotStationCount = 0;
	image->getSaveNumber();
	robot->progress = 10;
	robot->progressSpeed = 2.0;
	calibrated = trans->loadCalibration();
	stationSpotted = 0;
	state = STATE_IDLE;
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
			if (robot->rotateByAngle()){
				robot->lightsOn();
				state = STATE_ADJUST;
			}
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
				robot->movePtu(-314,0);
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

void depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (state ==  STATE_UNDOCK_MOVE && ptuTilt > -5)
	{
		int len = msg->height*msg->width;

		//TODO assuming same params for depth and RGB
		float vx = 1/trans->fc[0];
		float vy = 1/trans->fc[1];
		float cx = -trans->cc[0];
		float cy = -trans->cc[1];
		int width = msg->width;
		int height = msg->height;
		float fx = (1+cx)*vx;
		float fy = (1+cy)*vy;
		float lx = (width+cx)*vx;
		float ly = (height+cy)*vy;

		float x[len+1];
		float y[len+1];
		float z[len+1];
		float d[len+1];
		float di,psi,ix,iy,iz;
		int cnt = 0;
		di=psi=0;
		CTimer timer;
		timer.reset();
		timer.start();
		psi =  ptuTilt;
		float minDist = 100;
		for (float h = fy;h<ly;h+=vy)
		{
			for (float w = fx;w<lx;w+=vx)
			{
				di = *((float*)(&msg->data[4*cnt]));
				if (di > 0.05 && cnt%10 == 0 && di < 3.0){
					ix = di*(cos(psi)-sin(psi)*h);
					iy = -w*di;
					iz = -di*(sin(psi)+cos(psi)*h)+1.68;
					if (iz > 0.2 && iz < 1.9 && fabs(iy) < 0.3 && ix < minDist) minDist = ix;
					//printf("%.3f %.3f %.3f\n",ix,iy,iz);
				}
				cnt++;
			}
		}
		robot->setObstacleDistance(minDist); 
		//printf("Obstacle detected at %.3f\n",minDist);
	}else{
		robot->setObstacleDistance(100.0); 
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
	if (state == STATE_CALIBRATE_INIT)
	{
		if (robot->wait()) state = STATE_CALIBRATE;
	}
	if (state == STATE_UNDOCK_INIT)
	{
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
					if (robot->approach(station,station.x) && stationSpotted > 10) state = STATE_ADJUST;
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
						robot->lightsOff();
						sprintf(posString,"Robot position after undock: %f %f %f %f",own.x,own.y,own.z,own.yaw);
						printf("%s\n",response);
						success = true;
						robot->injectPosition(own.z,own.x,own.yaw);
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
							robot->lightsOff();
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
							robot->lightsOff();
							bool headOff = false;
							nh->getParam("charging/headOff",headOff);
							if (headOff){
								state = STATE_HEAD_OFF;
								robot->headOff(10);
							}else{
								state = STATE_DOCKING_SUCCESS;
							}
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
	subscribe();
	timer.reset();
	timer.start();
	timeOut = 120000;
	move_base_msgs::MoveBaseResult result;
	/*if (state == STATE_HEAD_RESTART){
		sprintf(response,"Sorry, have to wake up the head first. Try again in 30 seconds.\n");
		state = STATE_REJECTED;
		as->setAborted(result);
		robot->lightsOff();
		return;
	}*/
	if (calibrated==false){
		sprintf(response,"Cannot approach the charging station because the docking is not calibrated.\nRead the readme file on how to perform calibration procedure.");
		state = STATE_REJECTED;
	}else{
		robot->progress = 10;
		state = STATE_INIT;
		robot->lightsOn();
	}
	if (state == STATE_REJECTED){
		dockingServer->setAborted(result);
		robot->lightsOff();
		unsubscribe();
		return;
	}
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT && state != STATE_PREEMPTED){
		usleep(200000);
	}
	unsubscribe();
	if (state == STATE_PREEMPTED){
		dockingServer->setPreempted(result);
		state = STATE_IDLE;
		robot->lightsOff();
		return;
	}else if (state == STATE_ABORTED){
		dockingServer->setAborted(result);
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
	if (chargerDetected)robot->movePtu(-314,0); else robot->movePtu(0,0);
	robot->lightsOff();
}


void undockingServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, DockingServer* as)
{	
	initComponents();
	subscribe();
	timeOut = 120000;
	timer.reset();
	timer.start();
	move_base_msgs::MoveBaseResult result;
	/*if (state == STATE_HEAD_RESTART){
		sprintf(response,"Sorry, have to wake up the head first. Try again in 30 seconds.\n");
		state = STATE_REJECTED;
		as->setAborted(result);
		robot->lightsOff();
		return;
	}*/
	if (chargerDetected == false){
		state = STATE_REJECTED;
		sprintf(response,"Cannot undock because not on the charging station.");
	}else{
		bool headOn = true;
		nh->getParam("/EBC/MCU_24V_Enabled",headOn);
		if (headOn)
		{
			robot->wait(100);
			state = STATE_UNDOCK_INIT;
		}else{
			robot->headOn(130);
			state = STATE_HEAD_ON;
		}
		robot->movePtu(-314,53);
	}
	if (state == STATE_REJECTED){
		undockingServer->setAborted(result);
		robot->lightsOff();
		unsubscribe();
		return;
	}
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT && state != STATE_PREEMPTED){
		usleep(200000);
	}
	unsubscribe();
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
	if (chargerDetected)robot->movePtu(-314,0); else robot->movePtu(0,0);
	robot->lightsOff();
}

void actionServerCallback(const scitos_docking::ChargingGoalConstPtr& goal, Server* as)
{
	initComponents();
	subscribe();
	timer.reset();
	timer.start();
	timeOut = (int) goal->Timeout*1000;
	/*if (state == STATE_HEAD_RESTART){
		sprintf(response,"Sorry, have to wake up the head first. Try again in 30 seconds.\n");
		state = STATE_REJECTED;
		server->setAborted(result);
		robot->lightsOff();
		return;
	}*/
	if (goal->Command == "charge"){
		if (calibrated==false){
			sprintf(response,"Cannot approach the charging station because the docking is not calibrated.\nRead the readme file on how to perform calibration procedure.");
			state = STATE_REJECTED;
		}else{
			robot->progress = 10;
			state = STATE_INIT;
			robot->lightsOn();
		}
	}
	if (goal->Command == "test"){
		 robot->lightsOn();
		 ptupos = (int)goal->Timeout;
		 robot->movePtu(-314,0);
		 waitCycles = 0;
		 state = STATE_TEST1;
	}
	if (goal->Command == "calibrate"){
		state = STATE_CALIBRATE_INIT;
		robot->movePtu(0,0);
		robot->wait(100);
		robot->measure(NULL,NULL,maxMeasurements);
		robot->lightsOn();
	}
	if (goal->Command == "headon") robot->headOn(100);
	if (goal->Command == "headoff") robot->headOff(100);
	if (goal->Command == "undock"){
		if (chargerDetected == false){
			state = STATE_REJECTED;
			sprintf(response,"Cannot undock because not on the charging station.");
		}else{
			bool headOn = true;
			nh->getParam("/EBC/MCU_24V_Enabled",headOn);
			if (headOn)
			{
				robot->wait(100);
				state = STATE_UNDOCK_INIT;
			}else{
				robot->headOn(130);
				state = STATE_HEAD_ON;
			}
			robot->movePtu(-314,53);
		}
	}
	if (state == STATE_REJECTED){
		result.Message = response;
		server->setAborted(result);
		robot->lightsOff();
		unsubscribe();
		return;
	}
	while (state != STATE_IDLE && state != STATE_ABORTED && state != STATE_TIMEOUT && state != STATE_PREEMPTED){
		usleep(200000);
		server->publishFeedback(feedback);
	}
	result.Message = response;
	unsubscribe();
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
	if (chargerDetected)robot->movePtu(-314,0); else robot->movePtu(0,0);
	robot->lightsOff();
}

void ptuCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
	for (int i = 0;i<2;i++){
		if (msg->name[i] == "pan")  robot->ptuPan = ptuPan = msg->position[i];
		if (msg->name[i] == "tilt") ptuTilt = msg->position[i];
	}
}

void joyCallback(const scitos_teleop::action_buttons::ConstPtr &msg)
{
	if (msg->X && state == STATE_IDLE && server->isActive() == false && undockingServer->isActive() == false && dockingServer->isActive()==false){
		if (chargerDetected){
			bool headOn = true;
			nh->getParam("/EBC/MCU_24V_Enabled",headOn);
			if (headOn)
			{
				robot->wait(100);
				state = STATE_UNDOCK_INIT;
			}else{
				robot->headOn(130);
				state = STATE_HEAD_ON;
			}
			robot->movePtu(-314,53);
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
		if (state != STATE_IDLE && state != STATE_HEAD_ON) robot->moveHead();
		ros::spinOnce();
		if (positionUpdate && robot->poseSet == false) robot->injectPosition(); 
		usleep(30000);
		if (state!=lastState){
			sprintf(status,"Charging service is %s",stateStr[state]);
			ROS_INFO("Charging service is %s",stateStr[state]);
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


void subscribe() {
	ROS_INFO("subscribing to camera topics");
	image_transport::ImageTransport it(*nh);
	subim = it.subscribe("head_xtion/rgb/image_mono", 1, imageCallback);
	subdepth = it.subscribe("head_xtion/depth/image_rect", 1, depthCallback);
	subcamera = nh->subscribe("head_xtion/rgb/camera_info", 1,cameraInfoCallBack);
}


void unsubscribe() {
	ROS_INFO("unsubscribing from camera topics");
	subim.shutdown();
	subdepth.shutdown();
	subcamera.shutdown();
}


int main(int argc,char* argv[])
{
	ros::init(argc, argv, "charging");
	nh = new ros::NodeHandle;
	robot = new CChargingActions(nh);

	dump = new CDump(NULL,256,1000000);
	image = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	trans = new CTransformation(circleDiameter,nh);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	initComponents();
	success = false;
	nh->param("positionUpdate",positionUpdate,false);

	image_transport::ImageTransport it(*nh);
        imdebug = it.advertise("/charging/processedimage", 1);

	server = new Server(*nh, "chargingServer", boost::bind(&actionServerCallback, _1, server), false);
	dockingServer = new DockingServer(*nh, "docking", boost::bind(&dockingServerCallback, _1, dockingServer), false);
	undockingServer = new DockingServer(*nh, "undocking", boost::bind(&undockingServerCallback, _1, undockingServer), false);

	joy_sub_ = nh->subscribe("/teleop_joystick/action_buttons", 10, joyCallback);
	subodo = nh->subscribe("odom", 1, odomCallback);
	subcharger = nh->subscribe("battery_state", 1, batteryCallBack);
	ptu_sub_ = nh->subscribe("/ptu/state", 10, ptuCallback);
	robot_pose = nh->subscribe("/robot_pose", 1000, poseCallback);


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
