#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>

ros::Publisher pub_cmd_vel;
double l_scale_, a_scale_;
  

geometry_msgs::Twist t;

void controlCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  t.linear.x = 0.9*t.linear.x + 0.1*l_scale_ * msg->axes[1];
  t.angular.z = 0.5*t.angular.z + 0.5*a_scale_ * msg->axes[0];
	if(msg->header.frame_id == "stop") {
		t.linear.x = 0.0;
		t.angular.z = 0.0;
	}
	pub_cmd_vel.publish(t);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_base");

  ros::NodeHandle n("teleop_base");
  n.param("scale_angular", a_scale_, 0.8);
  n.param("scale_linear", l_scale_, 0.8);

  ros::Subscriber sub = n.subscribe("/teleop_joystick/joy", 1000, controlCallback);
  pub_cmd_vel = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  
  ros::spin();

  return 0;
}
