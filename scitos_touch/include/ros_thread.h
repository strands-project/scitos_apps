#ifndef ROS_THREAD_H
#define ROS_THREAD_H

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

class RosThread
{
public:		
		RosThread(std::string handle) {
				ros::NodeHandle n(handle);		
				reset_client = n.serviceClient<scitos_msgs::ResetMotorStop>(RESET_MOTORS);
	  		emergency_client = n.serviceClient<scitos_msgs::EmergencyStop>(EMERGENCY_STOP);
				sub = n.subscribe(BUMPER, 1000, &RosThread::bumperCallback, this);
		}

		void start() {  
	    	m_Thread = boost::thread(&RosThread::spin, this);  
		}

		void stop() {
				ros::shutdown();
		}

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

		void bumperCallback(const std_msgs::Bool::ConstPtr& msg) {
				boost::lock_guard<boost::mutex> lock(bumper_mut);
				motors_on = !msg->data;
		}

		bool isMotorsOn() {
				boost::lock_guard<boost::mutex> lock(bumper_mut);
				return motors_on;
		}

private:
		boost::thread m_Thread;
		boost::mutex service_mut, bumper_mut;
		bool motors_on;

		void spin() {
				ros::spin();
		}
	
		ros::ServiceClient reset_client, emergency_client;
    scitos_msgs::ResetMotorStop reset_srv;
    scitos_msgs::EmergencyStop emergency_srv;
		ros::Subscriber sub;

};


#endif // ROS_THREAD_H
