#include "CChargingActions.h"

CChargingActions::CChargingActions(ros::NodeHandle *n)
{
	timer.reset();
	timer.start();
	nh=n;
	cmd_vel = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	cmd_head = nh->advertise<sensor_msgs::JointState>("/head/commanded_state", 1);
	currentAngle = 0;
	head.name.resize(4);
        head.position.resize(4);
	warningLevel = .2;
}

CChargingActions::~CChargingActions()
{
}

float normalizeAngle(float a,float minimal=-M_PI)
{
	while (a > +2*M_PI+minimal) a-=2*M_PI;
	while (a < minimal) a+=2*M_PI;
	return a;
}

void CChargingActions::controlHead(int lids,int tilt, int pan)
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

void CChargingActions::updatePosition(const nav_msgs::Odometry &msg)
{
	current = msg;
	currentAngle = tf::getYaw(current.pose.pose.orientation);
}

void CChargingActions::moveHead()
{
	cmd_head.publish(head);
}

bool CChargingActions::rotateByAngle(float angle)
{
	static float desiredAngle;
	static float turnAngle;
	if (angle != .0){
		 desiredAngle = normalizeAngle(currentAngle+angle);
		 turnAngle = angle;
	}
	base_cmd.linear.x = 0; 
	base_cmd.angular.z = normalizeAngle(desiredAngle-currentAngle);
	cmd_vel.publish(base_cmd);
	progress = 100*(1-fabs(normalizeAngle(desiredAngle-currentAngle))/fabs(turnAngle));
	return fabs(normalizeAngle(desiredAngle-currentAngle)) < 0.05;
}

bool CChargingActions::moveByDistance(float distance)
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
	base_cmd.linear.x = speedSign*fmin(desiredDistance - travelledDistance,0.5);
	base_cmd.angular.z = 0; 
	cmd_vel.publish(base_cmd);
	progress = 100*travelledDistance/desiredDistance;
	return (fabs(desiredDistance - travelledDistance) < 0.02);
}

bool CChargingActions::search()
{
	base_cmd.linear.x = 0; 
	base_cmd.angular.z = 0.2;
	cmd_vel.publish(base_cmd);
	controlHead(100,0,0);
	progress = normalizeAngle(currentAngle-lastAngle,0)/2/M_PI*100.0;
}

bool CChargingActions::measure(STrackedObject *o1,STrackedObject *o2,int count,bool ml)
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
	progress = 100*(posCount+30)/(maxCount+30);
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

void CChargingActions::initCharging(bool charge,int maxMeasurements)
{	
	controlHead(100,0,0);
	ros::spinOnce();
	progress = 10;
	if (charge){
		measure(NULL,NULL,maxMeasurements);	
	}else{
		lastAngle = currentAngle;
	}
}

bool CChargingActions::approach(STrackedObject station,float dist)
{
	static float distance;
	if (dist != 0.0) distance = fabs(dist);
	bool complete = false;
	float angle = atan2(station.y,station.x); 
	base_cmd.linear.x = fmin(fabs(station.x*cos(cos(cos(angle)))+0.2),1)*0.4;
	base_cmd.angular.z = atan2(station.y,station.x);
	if (station.x < 0.4){
		complete = true; 
		base_cmd.linear.x = base_cmd.angular.z = 0;
		startProg = station.y;
	}
	progress = 100*(distance-station.x+0.4)/(distance+0.4);
	cmd_vel.publish(base_cmd);
	return complete;
}

bool CChargingActions::adjust(STrackedObject station,float in)
{
	static float init;
	if (in != 0.0) init = fabs(in);
	base_cmd.linear.x = 0; 
	base_cmd.angular.z = atan2(station.y,station.x)*0.5;
	if (fabs(station.y) < 0.05){
		base_cmd.angular.z = 0;
		 return true;
	}
	progress = 100*((init-fabs(station.y))/init);
	cmd_vel.publish(base_cmd);
	return false;
}

bool CChargingActions::dock(STrackedObject station)
{
	bool complete = false;
	base_cmd.linear.x = (station.x - 5*fabs(station.y)+0.3)*0.2;
	if (fabs(station.y) > 0.02) base_cmd.linear.x = 0; 
	base_cmd.angular.z = station.y;
	base_cmd.angular.z = atan2(station.y,station.x);
	if (station.x < 0.025){
		complete = true;
		base_cmd.linear.x = base_cmd.angular.z = 0;
	}
	progress = 100*(startProg-station.x+0.025)/(startProg+0.025);
	cmd_vel.publish(base_cmd);
	return complete;
}

bool CChargingActions::wait(STrackedObject *own,STrackedObject station,bool chargerDetect)
{
	if (chargerDetect){
		measure(own,&station,-1);
		float prec = sqrt(own->x*own->x+own->y*own->y);
		ROS_INFO("Position of the robot relative to the charging station is %.3f %.3f %.3f, i.e. %.0f mm from the desired position->",own->x,own->y,own->z,prec*1000);
		controlHead(0,180,-10);
		base_cmd.linear.x = base_cmd.angular.z = 0;
		cmd_vel.publish(base_cmd);
		progress = 100;
		return true;
	}else{
		head.position[2] = 180;
		head.position[3] = 0;
		base_cmd.linear.x = base_cmd.angular.z = 0;
		cmd_vel.publish(base_cmd);
		if (measure(own,&station)){
			moveByDistance(-0.5);
			controlHead(100,0,0);
			float prec = sqrt(own->x*own->x+own->y*own->y+own->z*own->z);
			ROS_INFO("Position of the robot relative to the charging station is %.3f %.3f %.3f, i.e. %f mm from the desired position, but no charging signal received.",own->x,own->y,own->z,prec*1000);
			return true;
		}
	}
	return false;
}

bool CChargingActions::halt()
{
	base_cmd.linear.x = base_cmd.angular.z = 0;
	cmd_vel.publish(base_cmd);
}

bool CChargingActions::actionStuck()
{
	static int lastTimer;
	float progr = progress;
	int timi = timer.getTime();
	float filterFactor = 0.02;
	if (lastProgress > progr+5){
		progressSpeed = 2;
	}else{
		if (lastTimer < timi)	progressSpeed =  progressSpeed*(1-filterFactor)+filterFactor*(progr-lastProgress)/(timi-lastTimer)*1000;
		if (progressSpeed < 0) printf("KOO: %i %i %f %f\n",timi,lastTimer,progr,lastProgress);
	}
	lastTimer = timer.getTime();
	lastProgress = progr; 
	if (progressSpeed < 0.2) return true; else return false;
}
