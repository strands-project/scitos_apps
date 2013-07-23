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
#include <scitos_msgs/ChargerStatus.h>
#include <scitos_apps_msgs/Charging.h>
#include <tf/tf.h>

#define MAX_PATTERNS 10 

CTimer timer;
int timeOut = 120;
int  imageWidth= 640;
int  imageHeight = 480;
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
int maxFailures=30;
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
nav_msgs::Odometry lastOdo;
nav_msgs::Odometry desired;
float currentAngle = 0;
float desiredAngle = 0;
float desiredDistance = 0;

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
	"has successfully reached the charger",
	"has failed to reach the charger",
	"rotating to align with station",
	"aligning with station",
	"rotating to face the station",
	"leaving station",
	"preparing for autonomous run",
	"checking if not on the charger already",
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
	STATE_INIT,
	STATE_NUMBER
}EState;

EState state = STATE_ROTATE;

float normalizeAngle(float a)
{
	while (a > +M_PI) a-=2*M_PI;
	while (a < -M_PI) a+=2*M_PI;
	return a;
}

void chargerCallBack(const scitos_msgs::ChargerStatus &msg)
{
	chargerDetected = msg.charging;
	if (chargerDetected){
		head.name[0] ="EyeLidLeft";
		head.position[0] = 0;
		head.name[1] ="EyeLidRight";
		head.position[1] = 0;
		cmd_head.publish(head);
	}
	if (chargerDetected==false && state == STATE_INIT){
		state = STATE_SEARCH;
		head.name[0] ="EyeLidLeft";
		head.position[0] = 100;
		head.name[1] ="EyeLidRight";
		head.position[1] = 100;
		cmd_head.publish(head);
	}
}

void ptuCallback(const nav_msgs::Odometry &msg){

}

void odomCallback(const nav_msgs::Odometry &msg)
{

	current = msg;
	currentAngle = tf::getYaw(current.pose.pose.orientation);
	if (state == STATE_SEARCH){
			   base_cmd.linear.x = 0; 
			   base_cmd.angular.z = 0.2;
			   cmd_vel.publish(base_cmd);
	}	
	if (state == STATE_ROTATE){
		base_cmd.linear.x = 0; 
		base_cmd.angular.z = normalizeAngle(desiredAngle-currentAngle);
		cmd_vel.publish(base_cmd);
	   if (fabs(normalizeAngle(desiredAngle-currentAngle)) < 0.05){
		   state = STATE_MOVE_TO;
		   lastOdo = current;
	   }
	}
	if (state == STATE_ROTATE_BACK){
		base_cmd.linear.x = 0; 
		base_cmd.angular.z = normalizeAngle(desiredAngle-currentAngle);
		cmd_vel.publish(base_cmd);
	   if (fabs(normalizeAngle(desiredAngle-currentAngle)) < 0.05){
		   state = STATE_ADJUST;
		   lastOdo = current;
	   }
	}
	if (state == STATE_MOVE_TO){
		float x = (current.pose.pose.position.x-lastOdo.pose.pose.position.x);
		float y = (current.pose.pose.position.y-lastOdo.pose.pose.position.y);
		float travelledDistance = sqrt(x*x+y*y);
		base_cmd.linear.x = desiredDistance - travelledDistance;
		base_cmd.angular.z = 0; 
		cmd_vel.publish(base_cmd);
		if (fabs(desiredDistance - travelledDistance) < 0.02){
			 state = STATE_ROTATE_BACK;
			 if (rotateBy < 0) rotateBy = -M_PI/2; else rotateBy = M_PI/2;
			 head.position[0] = 0;
			 desiredAngle = normalizeAngle(currentAngle-rotateBy);
		}
	}
	if (state == STATE_RETRY){
		float x = (current.pose.pose.position.x-lastOdo.pose.pose.position.x);
		float y = (current.pose.pose.position.y-lastOdo.pose.pose.position.y);
		float travelledDistance = sqrt(x*x+y*y);
		base_cmd.linear.x = -(desiredDistance - travelledDistance);
		base_cmd.angular.z = 0; 
		cmd_vel.publish(base_cmd);
		if (fabs(desiredDistance - travelledDistance) < 0.02){
			 state = STATE_APPROACH;
			 if (rotateBy < 0) rotateBy = -M_PI/2; else rotateBy = M_PI/2;
			 head.position[0] = 0;
			 desiredAngle = normalizeAngle(currentAngle-rotateBy);
		}
	}
	if (state == STATE_UNDOCK_MOVE)
	{
		float x = (current.pose.pose.position.x-lastOdo.pose.pose.position.x);
		float y = (current.pose.pose.position.y-lastOdo.pose.pose.position.y);
		float travelledDistance = sqrt(x*x+y*y);
		base_cmd.linear.x = -(desiredDistance - travelledDistance);
		base_cmd.angular.z = 0; 
		cmd_vel.publish(base_cmd);
		if (fabs(desiredDistance - travelledDistance) < 0.02){
			 state = STATE_UNDOCK_ROTATE;
			 rotateBy = M_PI;
			 head.name[0] ="EyeLidLeft";
			 head.position[0] = 0;
			 desiredAngle = normalizeAngle(currentAngle-rotateBy);
		}
	}
	if (state == STATE_UNDOCK_ROTATE){
		
	}
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
	if (state == STATE_INIT){
		head.name[0] ="EyeLidLeft";
		head.position[0] = 100;
		head.name[1] ="EyeLidRight";
		head.position[1] = 100;
		cmd_head.publish(head);
	}
	if ((int) state < (int)STATE_WAIT){
		memcpy(image->data,(void*)&msg->data[0],msg->step*msg->height);
		for (int i = 0;i<numBots;i++){
			lastSegmentArray[i] = currentSegmentArray[i];
			currentSegmentArray[i] = detectorArray[i]->findSegment(image,lastSegmentArray[i]);
			if (currentSegmentArray[i].valid)objectArray[i] = trans->transform(currentSegmentArray[i],false);
		}
		memcpy((void*)&msg->data[0],image->data,msg->step*msg->height);
		imdebug.publish(msg);
		if (state == STATE_SEARCH){
			if (currentSegmentArray[2].valid && currentSegmentArray[1].valid && currentSegmentArray[0].valid){
				base_cmd.linear.x = 0; 
				base_cmd.angular.z = 0;
				cmd_vel.publish(base_cmd);
				state = STATE_APPROACH;
				failedToSpotStationCount = 0; 
			}
		}	
		if (currentSegmentArray[2].valid && currentSegmentArray[1].valid && currentSegmentArray[0].valid){
			failedToSpotStationCount = 0; 
			if (state == STATE_APPROACH){
				STrackedObject station = trans->getDock(objectArray);
				base_cmd.linear.x = fmax((station.x - 5*fabs(station.y)+0.2)*0.4,0);
				base_cmd.angular.z = station.y*0.5;
				if (station.x < 0.4){
					state = STATE_MEASURE;
					base_cmd.linear.x = base_cmd.angular.z = 0;
					myPos.x = myPos.y = myPos.z = 0;
					posCount = -30; 

				}
				cmd_vel.publish(base_cmd);
			}
			if (state == STATE_ADJUST){
				STrackedObject station = trans->getDock(objectArray);
				base_cmd.linear.x = 0; 
				base_cmd.angular.z = atan2(station.y,station.x);
				if (fabs(station.y) < 0.05){
					state = STATE_MEASURE;
					base_cmd.linear.x = base_cmd.angular.z = 0;
					myPos.x = myPos.y = myPos.z = 0;
					posCount = -30; 
				}
				cmd_vel.publish(base_cmd);
			}

			if (state == STATE_DOCK){
				STrackedObject station = trans->getDock(objectArray);
				//if (station.z < 1.0) state = STATE_FINISHED;
				base_cmd.linear.x = (station.x - 5*fabs(station.y)+0.3)*0.2;
				if (fabs(station.y) > 0.02) base_cmd.linear.x = 0; 
				base_cmd.angular.z = station.y;
				if (station.x < 0.025){
					state = STATE_WAIT;
					posCount = -30;
					base_cmd.linear.x = base_cmd.angular.z = 0;
					myPos.x = myPos.y = myPos.z = 0;
				}
				cmd_vel.publish(base_cmd);
				head.position[0] = 0;
			}
			if (state == STATE_MEASURE){
				STrackedObject station = trans->getOwnPosition(objectArray);
				if (posCount >=0){
					myPos.x += station.x;
					myPos.y += station.y; 
					myPos.z += station.z; 
				}
				posCount++;
				head.name[0] ="EyeLidLeft";
				head.position[0] = posCount;
				head.name[1] ="EyeLidRight";
				head.position[1] = posCount;
				if (posCount > 100){
					head.name[0] ="HeadPan";
					head.name[1] ="ignore";
					head.position[1] = 0;
					myPos.x = myPos.x/posCount;
					myPos.y = myPos.y/posCount;
					myPos.z = myPos.z/posCount;
					//printf("POS: %f %f %f\n",myPos.x,myPos.y,myPos.z);
					desiredDistance = fabs(myPos.x);
					if (fabs(myPos.x) < 0.05) state = STATE_DOCK; else state = STATE_ROTATE;
					rotateBy = M_PI/2;
					if (myPos.x > 0) rotateBy = -rotateBy;
					//rotateBy += sin(myPos.x/myPos.z);
					rotateBy += sin((myPos.x+trans->ownOffset.x)/(myPos.z+trans->ownOffset.z));
					head.position[0] = -rotateBy*180/M_PI;
					desiredAngle = normalizeAngle(currentAngle+rotateBy);
				}
			}
			if (state == STATE_CALIBRATE){
				trans->clearOffsets();
				STrackedObject own = trans->getOwnPosition(objectArray);
				STrackedObject station = trans->getDock(objectArray);
				if (posCount >=0){
					myPos.x += own.x;
					myPos.y += own.y; 
					myPos.z += own.z; 
					dockPos.x += station.x;
					dockPos.y += station.y; 
					dockPos.z += station.z; 
				}
				posCount++;
				head.name[0] ="EyeLidLeft";
				head.position[0] = posCount;
				head.name[1] ="EyeLidRight";
				head.position[1] = posCount;
				if (posCount > 100){
					state = STATE_SUCCESS;
					myPos.x = myPos.x/posCount;
					myPos.y = myPos.y/posCount;
					myPos.z = myPos.z/posCount;
					dockPos.x = dockPos.x/posCount;
					dockPos.y = dockPos.y/posCount;
					dockPos.z = dockPos.z/posCount;
					nh->setParam("/charging/ownOffsetX", myPos.x);
					nh->setParam("/charging/ownOffsetY", myPos.y);
					nh->setParam("/charging/ownOffsetZ", myPos.z);
					nh->setParam("/charging/dockOffsetX", dockPos.x);
					nh->setParam("/charging/dockOffsetY", dockPos.y);
					nh->setParam("/charging/dockOffsetZ", dockPos.z);
				}
			}

			if (state == STATE_WAIT){
				STrackedObject station = trans->getOwnPosition(objectArray);
				if (posCount >=0){
					myPos.x += station.x;
					myPos.y += station.y; 
					myPos.z += station.z; 
				}
				posCount++;
				head.name[0] ="HeadPan";
				head.position[0] = 180;
				head.name[1] ="none";
				head.position[1] = 0.5;
				//if (posCount%100==0)printf("Waiting %i\n",posCount); 
				if (posCount > 200){
					head.name[0] ="HeadPan";
					head.name[1] ="ignore";
					head.position[1] = 0;
					myPos.x = myPos.x/posCount;
					myPos.y = myPos.y/posCount;
					myPos.z = myPos.z/posCount;
					ROS_INFO("Position of the robot relative to the charging station is %f %f %f\n",myPos.x,myPos.y,myPos.z);
					desiredDistance = 0.5;
					state = STATE_RETRY;
					lastOdo = current;
				}
			}
		}else{
			if (state != STATE_SEARCH) failedToSpotStationCount++;
		}
		if (failedToSpotStationCount > maxFailures) state=STATE_FAILURE; 
	}
}

int initComponents(){
	failedToSpotStationCount=0;
	image->getSaveNumber();
	state = STATE_IDLE;
	head.name.resize(2);
        head.position.resize(2);

        ptu.velocity.resize(2);
        ptu.position.resize(2);
        ptu.name.resize(2);
	posCount = -30; 

	nh->getParam("/charging/ownOffsetX", trans->ownOffset.x);
	nh->getParam("/charging/ownOffsetY", trans->ownOffset.y);
	nh->getParam("/charging/ownOffsetZ", trans->ownOffset.z);
	nh->getParam("/charging/dockOffsetX", trans->dockOffset.x);
	nh->getParam("/charging/dockOffsetY", trans->dockOffset.y);
	nh->getParam("/charging/dockOffsetZ", trans->dockOffset.z);

	myPos.x = myPos.y = myPos.z = 0;
	dockPos.x = dockPos.y = dockPos.z = 0;
}

bool receiveCommands(scitos_apps_msgs::Charging::Request  &req, scitos_apps_msgs::Charging::Response &res)
{
//	std::cout << "Request " << req.chargeCommand << " Timeout " << (int)req.chargeTimeout << std::endl;
	initComponents();
	if (req.chargeCommand == "charge") state = STATE_INIT;
	if (req.chargeCommand == "calibrate") state = STATE_CALIBRATE;
	timer.start();
	timer.reset();
	timeOut = (int)req.chargeTimeout;
	int a = 0;
	EState lastState = STATE_IDLE;
	while (ros::ok() && state != STATE_SUCCESS && state!=STATE_FAILURE && timer.getTime() < timeOut){
		cmd_head.publish(head);
		ros::spinOnce();
		usleep(10000);
		if (state!=lastState) ROS_INFO("Charging service is %s",stateStr[state]);
		if (chargerDetected && state!= STATE_CALIBRATE) state = STATE_SUCCESS;
		lastState = state;
	}
	char response[100];
	sprintf(response,"The robot %s.",stateStr[state]);
	res.chargeResult = response;
	return true;
}

int main(int argc,char* argv[])
{
	ros::init(argc, argv, "charging");
	nh = new ros::NodeHandle;
	image_transport::ImageTransport it(*nh);
	cmd_vel = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	//cmd_vel = nh->advertise<geometry_msgs::Twist>("/none", 1);
	cmd_head = nh->advertise<sensor_msgs::JointState>("/head/commanded_state", 1);
	cmd_ptu = nh->advertise<sensor_msgs::JointState>("/ptu/cmd", 2);

	dump = new CDump(NULL,256,1000000);
	image = new CRawImage(imageWidth,imageHeight);
	trans = new CTransformation(imageWidth,imageHeight,circleDiameter,true);
	for (int i = 0;i<MAX_PATTERNS;i++) detectorArray[i] = new CCircleDetect(imageWidth,imageHeight);

	initComponents();
	image_transport::Subscriber subim = it.subscribe("head_xtion/rgb/image_color", 1, imageCallback);
        imdebug = it.advertise("/charging/processedimage", 1);
	ros::Subscriber subodo = nh->subscribe("odom", 1, odomCallback);
	ros::Subscriber subcharger = nh->subscribe("charger_status", 1, chargerCallBack);
	ros::ServiceServer service = nh->advertiseService("chargingSrv", receiveCommands);

	timer.start();
	timer.reset();
	int a = 0;
	while (ros::ok()){
		/*ptu.name[0] ="pan";
		ptu.position[0] = 1.5;
		ptu.velocity[0] = 1.0;
		ptu.effort[0] = 1.0;
		ptu.name[1] ="tilt";
		ptu.position[1] = 0.0;
		ptu.velocity[1] = 1.0;
		ptu.effort[1] = 1.0;
		cmd_ptu.publish(ptu);*/
		ros::spinOnce();
		state = STATE_IDLE;
		usleep(10000);
		if (a++%200 == 0)ROS_INFO("Charging service is %s",stateStr[state]);
	}
	delete image;
	for (int i = 0;i<MAX_PATTERNS;i++) delete detectorArray[i];
	delete trans;
	return 0;
}

