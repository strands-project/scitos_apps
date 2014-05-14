#include <ros/ros.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/server/simple_action_server.h>
#include "CDoorDetection.h"

typedef actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> Server;
Server *server;
ros::Subscriber scan_sub;
ros::Subscriber robot_pose;

float goalPositionTolerance = 0.09;
float minRotate = 0.3; //minimum amount to rotate -- 0.3 was needed at BHAM to prevent getting stuck on carpet
float defaultMaxDistance = 3.0;
float maxDistance = defaultMaxDistance;		//max range taken into consideration
float defaultSpeed = 0.15;		//default forward speed of the robot
float baseRadius = 0.31;
int passCounterLimit = 10;		//measurements 
int passCounter = 0;			//measurements 
int maxMisdetections = 50;		//decides when door not detected
int misdetections = 0;
bool debug = true;

float previousMaxDistance = 0; 
int increasingGoalDistance = 0; //pose updates
int increasingGoalDistanceThreshold = 15; //pose updates
bool longRangeMode = false;

typedef enum{
	IDLE,
	TURNING,
	DETECT,
	APPROACH,
	ADJUST,
	PASS,
	LEAVE,
	STOPPING,
	PREEMPTED,
	SUCCESS,
	FAIL
}EClimbState;

EClimbState state = IDLE;
ros::Publisher cmd_vel;
geometry_msgs::Twist base_cmd;
float goalX;
float goalY;

float boundRotation(const float & _rot) {
	if(fabs(_rot) < minRotate) {
		if(_rot > 0) {
			return minRotate;
		}
		else {
			return -minRotate;	
		}
	}
	else {
		return _rot;
	}

}

void poseCallback(const geometry_msgs::Pose::ConstPtr& msg)
{

	if (state == TURNING || state == APPROACH || state == ADJUST || state == PASS || state == LEAVE) {
		float rX,rY;
		rX=rY=0;

		// difference in x y between current and target
		rX = goalX - msg->position.x;
		rY = goalY - msg->position.y;

		// only check for doors between the pose and the target		
		maxDistance = fmin(sqrt(rX*rX+rY*rY), maxDistance);

		if (debug) printf("max distance to goal %f\n", maxDistance);

		// if we're close enough to the goal, regardless of other counts
		if(maxDistance <= goalPositionTolerance) {
			printf("within range of target, stopping");			
			state = LEAVE;
		}
		

		// if we're not turning we should be getting closer to the goal
		if(state != TURNING) {

			// are we getting closer to the goal?
			float goalDifference = previousMaxDistance - maxDistance;
			previousMaxDistance = maxDistance;
			
			if(goalDifference > 0) {
				if(++increasingGoalDistance > increasingGoalDistanceThreshold) {
					printf("heading away from goal, failing");			
					state = FAIL;
				}
			}
			else {
				increasingGoalDistance = 0;
			}

		}
		else { //if (state == TURNING){
			float currentAngle = tf::getYaw(msg->orientation);

			base_cmd.linear.x = 0; 
			currentAngle = (atan2(rY,rX)-currentAngle);

			while (currentAngle >= M_PI) currentAngle-= 2*M_PI;
			while (currentAngle < -M_PI) currentAngle += 2*M_PI;

			base_cmd.angular.z = boundRotation(currentAngle*0.5);



			if (fabs(currentAngle) < 0.1){
				base_cmd.angular.z = 0;
				state = APPROACH;
			} 

			cmd_vel.publish(base_cmd);
		}
	}

}

void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{

	if(debug) {
		if(state == LEAVE) {
			printf("LEAVE\n");
		}
		else if(state == APPROACH) {
			printf("APPROACH\n");
		}
		else if(state == ADJUST) {
			printf("ADJUST\n");
		}
		else if(state == PASS) {
			printf("PASS\n");
		}

	}


	if (state == APPROACH || state == ADJUST || state == PASS || state == LEAVE){
		
		base_cmd.linear.x = 0; 
		base_cmd.angular.z = 0; 

		if (state == APPROACH || state == DETECT || state == ADJUST){


			// sometimes this helps get through when a bit stuck... 
			if (misdetections > maxMisdetections/2) {
				if (debug) printf("Extending range to try to find lost door\n");
				longRangeMode = true;
			} 


			// how far away should we look for doors? -- default to distance to goal
			float detectDistance = longRangeMode ? defaultMaxDistance : maxDistance;
			SDoor door = detectDoor(scan_msg->ranges,scan_msg->angle_min,scan_msg->angle_increment,scan_msg->ranges.size(),detectDistance);

			if (door.found){
				misdetections=0;
				if (state == APPROACH){
					if (debug) printf("Moving to %f %f ",door.auxX,door.auxY);
					if (door.auxX < 0.05) passCounter++; else passCounter=0;
					if (passCounter >  passCounterLimit){
						state = ADJUST;
						passCounter =0;
					}
					if (debug) printf("\n");
					base_cmd.linear.x = fmax(fmin(defaultSpeed,door.auxX),0); 
					base_cmd.angular.z = boundRotation(door.auxY);
					cmd_vel.publish(base_cmd);
				}
				if (state == ADJUST){
					if (debug) printf("Turning %f \n",door.doorCentreY);
					if (fabs(door.doorCentreY) < 0.05) passCounter++; else passCounter=0;
					if (passCounter >  passCounterLimit) state = PASS;
					base_cmd.linear.x = 0;
					base_cmd.angular.z = boundRotation(door.doorCentreY); 
					cmd_vel.publish(base_cmd);
				}

			}
			else {
				misdetections++;
			}

			if (state == DETECT){
				if (door.found) passCounter++; else passCounter=0;
				if (passCounter >  passCounterLimit) state = LEAVE;
			}
		}
		
		if (state == PASS){
			float leftMinim,rightMinim;
			leftMinim=rightMinim = 100;


			for (int i = 0;i<scan_msg->ranges.size();i++){
				float angle = scan_msg->angle_min+i*scan_msg->angle_increment;
				float d = fmin(scan_msg->ranges[i],maxDistance);
				float x = d*cos(angle);
				float y = d*sin(angle);
				if (x < 0.55){
					if (i<scan_msg->ranges.size()/2){
						if (rightMinim>-y) rightMinim = -y;
					}else{
						if (leftMinim>y) leftMinim = y;
					}
				} 
			}
			leftMinim-=baseRadius;
			rightMinim-=baseRadius;
			if (debug) printf("Obstacles %f %f \n",leftMinim,rightMinim);
			base_cmd.linear.x = fmax(fmin(3*fmin(rightMinim,leftMinim),defaultSpeed),0); 
			base_cmd.angular.z = boundRotation(2*(leftMinim-rightMinim));
			if (leftMinim > 0.1 && rightMinim >0.1){
				if (debug) printf("HAPPY %f %f %d\n",leftMinim,rightMinim, passCounter);
				base_cmd.angular.z =0;
				base_cmd.linear.x = 0.2;
				passCounter++;
			}else{
				passCounter = -30;
			}
			if (passCounter > passCounterLimit) state = LEAVE;
		
			cmd_vel.publish(base_cmd);
		}
	}
}

void actionServerCallback(const move_base_msgs::MoveBaseGoalConstPtr& goal, Server* as)
{
	move_base_msgs::MoveBaseResult result;
	goalX = goal->target_pose.pose.position.x;
	goalY = goal->target_pose.pose.position.y;
	
	// reset variables
	misdetections = 0;
	maxDistance = defaultMaxDistance;
	previousMaxDistance = defaultMaxDistance;
	increasingGoalDistance = 0;
	longRangeMode = false;

	state = TURNING;

	ros::Rate r(50); //hz

	if (goalX == 0 && goalY == 0) state = APPROACH;
	while (state == TURNING || state == DETECT || state == APPROACH || state == ADJUST || state == PASS || state == LEAVE){
		
		if (misdetections > maxMisdetections || state == LEAVE){
			if (state == LEAVE) {
				state = SUCCESS;
			}
			else {
				state = FAIL;
			}
		}

		r.sleep();
	}
	
	if (state == SUCCESS) server->setSucceeded(result);
	if (state == FAIL) server->setAborted(result);
	if (state == PREEMPTED) server->setPreempted(result);
	state = STOPPING;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "doorPassing");
	ros::NodeHandle n;
	cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	robot_pose = n.subscribe("/robot_pose", 1000, poseCallback);
	server = new Server(n, "doorPassing", boost::bind(&actionServerCallback, _1, server), false);
	server->start();
	scan_sub = n.subscribe("scan", 100, scanCallback);

	ros::Rate r(30); //hz
	while (ros::ok()){
		if (server->isPreemptRequested() && state != IDLE) state = PREEMPTED;
		if (state == STOPPING)
		{
			base_cmd.linear.x = base_cmd.angular.z = 0;
			cmd_vel.publish(base_cmd);
			state = IDLE;
		} 
		ros::spinOnce();
		r.sleep();
	}
}

