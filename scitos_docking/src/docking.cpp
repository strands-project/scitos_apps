#include <stdlib.h>
#include "CDump.h"
#include "CTimer.h"
#include "CCircleDetect.h"
#include "CTransformation.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <scitos_msgs/BatteryState.h>
#include <scitos_apps_msgs/Charging.h>
#include <tf/tf.h>

#define MAX_PATTERNS 10 

bool calibrated = true;
float testSpeed = 0;
CTimer timer;
int timeOut = 120;
int  defaultImageWidth= 320;
int  defaultImageHeight = 240;
float circleDiameter = 0.05;
float rotateBy = 0;
bool chargerDetected = false;

ETransformType transformType = TRANSFORM_NONE;

ros::NodeHandle *nh;
geometry_msgs::Twist base_cmd;
sensor_msgs::JointState head;
sensor_msgs::JointState ptu;
ros::Publisher cmd_vel;
ros::Publisher cmd_head;
ros::Publisher cmd_ptu;
image_transport::Publisher imdebug;

int failedToSpotStationCount=0;
int maxFailures=60;
int numBots = 3;
int posCount = -30;
STrackedObject myPos,dockPos;
TLogModule module = LOG_MODULE_MAIN;
int numSaved = 0;
CRawImage *image;

CCircleDetect *detectorArray[MAX_PATTERNS];
STrackedObject objectArray[MAX_PATTERNS];
SSegment currentSegmentArray[MAX_PATTERNS];
SSegment lastSegmentArray[MAX_PATTERNS];
CTransformation *trans;

nav_msgs::Odometry current;
float currentAngle = 0;
float lastAngle = 0;

const char* stateStr[] ={
	"measuring the robot positition relatively to the ROBOT STATION banner",
	"searching for the docking",
	"approaching the station",
	"measuring position",
	"attempting to reach the station",
	"adjusting robot position",
	"waiting for charger signal",
	"retrying to reach charger attempt",
	"idle",
	"happy",
	"sad",
	"rotating to align with station",
	"aligning with station",
	"rotating to face the station",
	"leaving station",
	"preparing for autonomous run",
	"performing a test movement",
	"checking if not on the charger already",
	"testmove1",
	"testmove2",
	"in an unknown state"
};

typedef enum{
	STATE_CALIBRATE=0,
	STATE_SEARCH,
	STATE_APPROACH,
	STATE_MEASURE,
	STATE_DOCK,
	STATE_ADJUST,
	STATE_WAIT,
	STATE_RETRY,
	STATE_IDLE,
	STATE_SUCCESS,
	STATE_FAILURE,
	STATE_ROTATE,
	STATE_MOVE_TO,
	STATE_ROTATE_BACK,
	STATE_UNDOCK_MOVE,
	STATE_UNDOCK_ROTATE,
	STATE_TEST_MOVE,
	STATE_INIT,
	STATE_TEST1,
	STATE_TEST2,
	STATE_NUMBER
}EState;

EState state = STATE_ROTATE;

float normalizeAngle(float a)
{
	while (a > +M_PI) a-=2*M_PI;
	while (a < -M_PI) a+=2*M_PI;
	return a;
}

void controlHead(int lids,int tilt, int pan)
{
	head.name[0] ="EyeLidRight";
	head.name[1] ="EyeLidLeft";
	head.name[2] ="HeadPan";
	head.name[3] ="HeadTilt";
	head.position[0] = lids;
	head.position[1] = lids;
	head.position[2] = tilt;
	head.position[3] = pan;
}

bool measure(STrackedObject *o1,STrackedObject *o2=NULL,int count = 0,bool ml=true)
{
	static bool moveLids;
	static int posCount;
	static int maxCount;
	static STrackedObject avgPos1,avgPos2,dummy;
	if (o1==NULL) o1 = &dummy;
	if (o2==NULL) o2 = &dummy;
	if (count > 0){
		moveLids = ml;
		avgPos1.x = avgPos1.y = avgPos1.z = avgPos2.x = avgPos2.y = avgPos2.z = 0;
	 	posCount = -30;
		maxCount = count;
	}
	if (count < 0){
		 maxCount = count;
		 if (posCount < 1) posCount = 1;
	}
	if (posCount >=0){
		avgPos1.x += o1->x;
		avgPos1.y += o1->y; 
		avgPos1.z += o1->z; 
		avgPos2.x += o2->x;
		avgPos2.y += o2->y; 
		avgPos2.z += o2->z; 
	}
	posCount++;
	if (moveLids){
		head.position[0] = fmax(100*posCount/maxCount,0);
		head.position[1] = fmax(100*posCount/maxCount,0);
	}else{
		head.position[0] = head.position[1] = 100;
	}
	if (posCount > maxCount){
		o1->x = avgPos1.x/posCount;
		o1->y = avgPos1.y/posCount;
		o1->z = avgPos1.z/posCount;
		o2->x = avgPos2.x/posCount;
		o2->y = avgPos2.y/posCount;
		o2->z = avgPos2.z/posCount;
		return true;
	}
	base_cmd.linear.x = base_cmd.angular.z = 0;
	return false;
}

void cameraInfoCallBack(const sensor_msgs::CameraInfo &msg)
{
	trans->updateParams(msg.K[2],msg.K[5],msg.K[0],msg.K[4]);
}


void batteryCallBack(const scitos_msgs::BatteryState &msg)
{
	chargerDetected = msg.charging;
}

bool rotateByAngle(float angle = .0)
{
	static float desiredAngle;
	if (angle != .0) desiredAngle = normalizeAngle(currentAngle+angle);
	base_cmd.linear.x = 0; 
	base_cmd.angular.z = normalizeAngle(desiredAngle-currentAngle);
	cmd_vel.publish(base_cmd);
	return fabs(normalizeAngle(desiredAngle-currentAngle)) < 0.05;
}

bool moveByDistance(float distance = .0)
{
	static nav_msgs::Odometry lastOdo;
	static float desiredDistance;
	static float speedSign;
	if (distance != .0){
		lastOdo = current;
		if (distance < 0) speedSign = -1; else speedSign = 1;
		desiredDistance= speedSign*distance;
	}
	float x = (current.pose.pose.position.x-lastOdo.pose.pose.position.x);
	float y = (current.pose.pose.position.y-lastOdo.pose.pose.position.y);
	float travelledDistance = sqrt(x*x+y*y);
	base_cmd.linear.x = speedSign*(desiredDistance - travelledDistance);
	base_cmd.angular.z = 0; 
	cmd_vel.publish(base_cmd);
	return (fabs(desiredDistance - travelledDistance) < 0.02);
}

void odomCallback(const nav_msgs::Odometry &msg)
{

	current = msg;
	currentAngle = tf::getYaw(current.pose.pose.orientation);
	switch (state){
		case STATE_SEARCH:
			base_cmd.linear.x = 0; 
			base_cmd.angular.z = 0.2;
			cmd_vel.publish(base_cmd);
			controlHead(100,0,0);
			break;

		case STATE_TEST1:
			moveByDistance(-0.2);
			state = STATE_TEST2;
			break;
		case STATE_TEST2:
			if (moveByDistance()) state = STATE_SUCCESS;
			break;
		case STATE_ROTATE:
			if (rotateByAngle()) state = STATE_MOVE_TO;
			controlHead(100,-rotateBy/M_PI*180,0);
			break;
		case STATE_ROTATE_BACK:
			if (rotateByAngle()) state = STATE_ADJUST;
			controlHead(100,0,0);
			break;
		case STATE_MOVE_TO:
			if (moveByDistance()){
				state = STATE_ROTATE_BACK; 
				if (rotateBy > 0) rotateBy = -M_PI/2; else rotateBy = M_PI/2;
				rotateByAngle(rotateBy);
			}
			break;
		case STATE_RETRY:
			if (moveByDistance()) state = STATE_APPROACH;
			controlHead(100,0,0);
			break;
		case STATE_UNDOCK_MOVE: 
			controlHead(100,180,0);
			if (moveByDistance()){
				rotateByAngle(M_PI+0.01);
				state = STATE_UNDOCK_ROTATE;
			}
			break;
		case STATE_UNDOCK_ROTATE:
			if (rotateByAngle()) state = STATE_SUCCESS;
			controlHead(100,0,0);
			lastAngle = currentAngle;
			break;
		case STATE_TEST_MOVE:
			base_cmd.linear.x = 0.3; 
			base_cmd.angular.z = 0.2;
			cmd_vel.publish(base_cmd);
			if (fabs(lastAngle-currentAngle) > 0.6) state = STATE_SUCCESS;	
			break;
	}
	cmd_head.publish(head);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	STrackedObject own,station;
	if (state == STATE_INIT){
		controlHead(100,0,0);
		ros::spinOnce();
		if (chargerDetected){
			state = STATE_WAIT;
			measure(NULL,NULL,100);	
		}else{
			state = STATE_SEARCH;
		} 
	}
	if ((int) state < (int)STATE_RETRY){
		if (image->bpp != msg->step/msg->width || image->width != msg->width || image->height != msg->height){
			delete image;
			image = new CRawImage(msg->width,msg->height,msg->step/msg->width);
			printf("Readjusting image size\n");
		}
		memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);


		//search image for circles
		for (int i = 0;i<numBots;i++){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
			if (currentSegmentArray[i].valid)objectArray[i] = trans->transform(currentSegmentArray[i]);
		}
		//and publish the result
		memcpy((void*)&msg->data[0],image->data,msg->step*msg->height);
		imdebug.publish(msg);
		//dockVisible ?	
		if (currentSegmentArray[2].valid && currentSegmentArray[1].valid && currentSegmentArray[0].valid){
			station = trans->getDock(objectArray);
			failedToSpotStationCount = 0;
			controlHead(100,180/M_PI*atan2(station.y,station.x),-180/M_PI*atan2(station.z-0.2,station.x));
			switch (state){

				case STATE_SEARCH:
				base_cmd.linear.x = 0; 
				base_cmd.angular.z = 0;
				cmd_vel.publish(base_cmd);
				state = STATE_APPROACH;
				break;

				case STATE_APPROACH:
				base_cmd.linear.x = fmin(fabs(station.x - 5*fabs(station.y)+0.2),1)*0.4;
				base_cmd.angular.z = station.y*0.5;
				if (station.x < 0.4) base_cmd.linear.x = 0; 
				if (station.x < 0.4 && fabs(station.y) < 0.05){
					state = STATE_MEASURE;
					measure(NULL,NULL,100);
				}
				cmd_vel.publish(base_cmd);
				break;
			  case STATE_ADJUST:
				base_cmd.linear.x = 0; 
				base_cmd.angular.z = atan2(station.y,station.x);
				if (fabs(station.y) < 0.05){
					state = STATE_MEASURE;
					measure(NULL,NULL,100);
				}
				cmd_vel.publish(base_cmd);
			  break;

			  case STATE_DOCK:
				base_cmd.linear.x = (station.x - 5*fabs(station.y)+0.3)*0.2;
				if (fabs(station.y) > 0.02) base_cmd.linear.x = 0; 
				base_cmd.angular.z = station.y;
				cmd_vel.publish(base_cmd);
				if (station.x < 0.025){
					state = STATE_WAIT;
					measure(NULL,NULL,400,false);	
				}
			break;
			case STATE_MEASURE:
			own = trans->getOwnPosition(objectArray);
			if (measure(&own)){
				if (fabs(own.x) < 0.05) state = STATE_DOCK; else state = STATE_ROTATE;
				rotateBy = M_PI/2;
				if (own.x > 0) rotateBy = -rotateBy;
				rotateBy += sin((own.x+trans->ownOffset.x)/(own.z+trans->ownOffset.z));
				rotateByAngle(rotateBy);
				moveByDistance(fabs(own.x));
			}
			break;
			case STATE_CALIBRATE:
				trans->clearOffsets();
				own = trans->getOwnPosition(objectArray);
				if (measure(&own,&station)){
					nh->setParam("/charging/ownOffsetX", own.x);
					nh->setParam("/charging/ownOffsetY", own.y);
					nh->setParam("/charging/ownOffsetZ", own.z);
					nh->setParam("/charging/dockOffsetX", station.x);
					nh->setParam("/charging/dockOffsetY", station.y);
					nh->setParam("/charging/dockOffsetZ", station.z);
					state = STATE_SUCCESS;
				}
			break;
			case STATE_WAIT:
				own = trans->getOwnPosition(objectArray);
				if (chargerDetected){
					measure(&own,&station,-1);
					ROS_INFO("Position of the robot relative to the charging station is %f %f %f\n",own.x,own.y,own.z);
					state = STATE_SUCCESS;
					controlHead(0,180,-10);
				}else{
					head.position[2] = 180;
					head.position[3] = 0;
					if (measure(&own,&station)){
						 state = STATE_RETRY;
						 moveByDistance(-0.5);
						 controlHead(100,0,0);
						 ROS_INFO("Position of the robot relative to the charging station is %f %f %f, but no charging signal received.\n",own.x,own.y,own.z);
					}
				}
				break;
			}
		}else{
			if (state != STATE_SEARCH) failedToSpotStationCount++;
		}
		if (failedToSpotStationCount > maxFailures) state=STATE_FAILURE; 
	}
	cmd_head.publish(head);
}

int initComponents(){
	failedToSpotStationCount=0;
	image->getSaveNumber();
	state = STATE_IDLE;

	head.name.resize(4);
        head.position.resize(4);

        ptu.velocity.resize(2);
        ptu.position.resize(2);
        ptu.name.resize(2);
	posCount = -30;
	calibrated = true; 
	calibrated = calibrated && nh->getParam("/charging/ownOffsetX", trans->ownOffset.x);
	calibrated = calibrated && nh->getParam("/charging/ownOffsetY", trans->ownOffset.y);
	calibrated = calibrated && nh->getParam("/charging/ownOffsetZ", trans->ownOffset.z);
	calibrated = calibrated && nh->getParam("/charging/dockOffsetX", trans->dockOffset.x);
	calibrated = calibrated && nh->getParam("/charging/dockOffsetY", trans->dockOffset.y);
	calibrated = calibrated && nh->getParam("/charging/dockOffsetZ", trans->dockOffset.z);
	myPos.x = myPos.y = myPos.z = 0;
	dockPos.x = dockPos.y = dockPos.z = 0;
}

bool receiveCommands(scitos_apps_msgs::Charging::Request  &req, scitos_apps_msgs::Charging::Response &res)
{
//	std::cout << "Request " << req.chargeCommand << " Timeout " << (int)req.chargeTimeout << std::endl;
	initComponents();
	char response[100];
	if (req.chargeCommand == "charge"){
		if (calibrated==false){
			ROS_ERROR("Cannot approach the charging station because the docking is not calibrated.\nRead the readme file on how to perform calibration procedure.");
			state = STATE_FAILURE;
		}else{
			state = STATE_INIT;
		}
	}
	if (req.chargeCommand == "test") state = STATE_TEST1;
	if (req.chargeCommand == "calibrate"){
		 state = STATE_CALIBRATE;
		 measure(NULL,NULL,200);
	}
	if (req.chargeCommand == "undock"){
		if (chargerDetected == false){
			state = STATE_FAILURE;
			sprintf(response,"Undocking aborted - the robot is not at the charging station.");
		}else{
			moveByDistance(-0.55);
			state = STATE_UNDOCK_MOVE;
			controlHead(100,180,0);
			cmd_head.publish(head);
		}
	}
	timer.reset();
	timer.start();
	timeOut = (int) req.chargeTimeout;
	int a = 0;
	EState lastState = STATE_IDLE;
	while (ros::ok() && state != STATE_SUCCESS && state!=STATE_FAILURE && timer.getTime() < timeOut){
		ros::spinOnce();
		usleep(10000);
		if (state!=lastState) ROS_INFO("Charging service is %s",stateStr[state]);
		lastState = state;
	}
	base_cmd.linear.x = base_cmd.angular.z = 0;
	cmd_vel.publish(base_cmd);
	ros::spinOnce();

	if (req.chargeCommand == "charge"){
		if (chargerDetected) sprintf(response,"The robot has successfully reached the charger."); else sprintf(response,"The robot has failed reached the charger.");
	}
	if (req.chargeCommand == "undock" && state == STATE_SUCCESS){
		 sprintf(response,"Undocking  successfully completed.");
	}
	res.chargeResult = response;
	return true;
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "charging");
	nh = new ros::NodeHandle;
	image_transport::ImageTransport it(*nh);
	cmd_vel = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	cmd_head = nh->advertise<sensor_msgs::JointState>("/head/commanded_state", 1);
	cmd_ptu = nh->advertise<sensor_msgs::JointState>("/ptu/cmd", 2);

	dump = new CDump(NULL,256,1000000);
	image = new CRawImage(defaultImageWidth,defaultImageHeight,4);
	trans = new CTransformation(circleDiameter);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(defaultImageWidth,defaultImageHeight);

	initComponents();
	image_transport::Subscriber subim = it.subscribe("head_xtion/rgb/image_mono", 1, imageCallback);
        imdebug = it.advertise("/charging/processedimage", 1);
	ros::Subscriber subodo = nh->subscribe("odom", 1, odomCallback);
	ros::Subscriber subcharger = nh->subscribe("battery_state", 1, batteryCallBack);
	ros::Subscriber subcamera = nh->subscribe("head_xtion/rgb/camera_info", 1,cameraInfoCallBack);
	ros::ServiceServer service = nh->advertiseService("chargingSrv", receiveCommands);

	timer.start();
	timer.reset();
	int a = 0;
	while (ros::ok()){
		ros::spinOnce();
		state = STATE_IDLE;
		usleep(10000);
		if (a++%200 == 0)ROS_INFO("Charging service state is %s",stateStr[state]);
	}
	delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}

