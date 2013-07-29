#ifndef ROS_COMM_H
#define ROS_COMM_H

#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <scitos_msgs/ResetMotorStop.h>
#include <scitos_msgs/EmergencyStop.h>

#define EMERGENCY_STOP "/emergency_stop"
#define RESET_MOTORS "/reset_motorstop"
#define BUMPER "/bumper"

class RosComm
{
public:		
		RosComm(std::string handle) {
				//Init nodehandle, services and subscriber				
				ros::NodeHandle n(handle);		
				reset_client = n.serviceClient<scitos_msgs::ResetMotorStop>(RESET_MOTORS);
	  		    emergency_client = n.serviceClient<scitos_msgs::EmergencyStop>(EMERGENCY_STOP);
				sub = n.subscribe(BUMPER, 1000, &RosComm::bumperCallback, this);
		}

		//Lets the spin() function run in a different thread
		void start() {
	    	m_Thread = boost::thread(&RosComm::spin, this);  
		}

		//Calls shutdown to end ros::spin() gracefully when Qt dies.
		void stop() {
				ros::shutdown();
		}

		//Calls the given service. Either stop or reset of motors.
		bool callService(std::string service) {
				boost::lock_guard<boost::mutex> lock(service_mut);
				if(strcmp(service.c_str(), EMERGENCY_STOP) == 0) {		
						if (!emergency_client.call(emergency_srv)) {
								ROS_ERROR("Failed to call service /emergency_stop");
								return false;
						} else {
								ROS_INFO("Motors stopped");
								return true;
						} 
				} else if(strcmp(service.c_str(), RESET_MOTORS) == 0) {
						if (!reset_client.call(reset_srv)) {
								ROS_ERROR("Failed to call service /reset_motorstop");
								return true;
						} else {
								ROS_INFO("Motors started");
								return true;
						} 
				}
				return false;
		}

		//Callback subscribed to the /bumper topic to determin current motor state.
		void bumperCallback(const std_msgs::Bool::ConstPtr& msg) {
				boost::lock_guard<boost::mutex> lock(bumper_mut);
				motors_on = !msg->data;
		}

		//Read last received motor state
		bool isMotorsOn() {
				boost::lock_guard<boost::mutex> lock(bumper_mut);
				return motors_on;
		}

private:
		boost::thread m_Thread;
		boost::mutex service_mut, bumper_mut;
		bool motors_on;

		//This runs in its own thread, see above.
		//Has to be because both ros and Qt have to run in a loop.
		void spin() {
				ros::spin();
		}
	
		ros::ServiceClient reset_client, emergency_client;
        scitos_msgs::ResetMotorStop reset_srv;
        scitos_msgs::EmergencyStop emergency_srv;
		ros::Subscriber sub;

};


#endif // ROS_COMM_H
