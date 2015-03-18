#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <ros/console.h>

#include "scitos_teleop/action_buttons.h"

#include <scitos_msgs/EnableMotors.h>
#include <scitos_msgs/ResetMotorStop.h>
#include <scitos_msgs/EmergencyStop.h>
#include <scitos_msgs/ResetBarrierStop.h>

ros::ServiceClient enable_client, reset_client, emergency_client, barrier_client;
scitos_msgs::EnableMotors enable_srv;
scitos_msgs::ResetMotorStop reset_srv;
scitos_msgs::EmergencyStop emergency_srv;
scitos_msgs::ResetBarrierStop barrier_srv;

ros::Publisher pub_joy, pub_buttons;

bool interrupt_broadcasting, sent_error;
bool last_buttons[4] = {0,0,0,0};

//Main callback handling the incomming joy_node messages
void controlCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    //Check for X mode
    if(msg->axes.size() != 8) {
        if(!sent_error) {
            ROS_ERROR("Rumblepad is running in D mode. Please switch to X mode.");
            ROS_ERROR("Pad will not work as long as it runs in the wrong mode.");
            sent_error = true;
        }
        return;
    }
    sent_error = false;

    //Publish action buttons
    scitos_teleop::action_buttons button_msg;
    if(msg->buttons[0] != last_buttons[0] || msg->buttons[1] != last_buttons[1] || msg->buttons[2] != last_buttons[2] || msg->buttons[3] != last_buttons[3]) {
        button_msg.A = msg->buttons[0];
        button_msg.B = msg->buttons[1];
        button_msg.X = msg->buttons[2];
        button_msg.Y = msg->buttons[3];
        pub_buttons.publish(button_msg);
    }
    last_buttons[0] = msg->buttons[0];
    last_buttons[1] = msg->buttons[1];
    last_buttons[2] = msg->buttons[2];
    last_buttons[3] = msg->buttons[3];

    //publish to /teleop_joystick/joy if deadman switch is held
    if(msg->buttons[4]) {
        interrupt_broadcasting = false;
        pub_joy.publish(msg);
    } else { //publish a msg containing zeros for scalar and angular velocity once if the deadman switch is released
        if(interrupt_broadcasting == false){
            sensor_msgs::Joy::Ptr& tmp((sensor_msgs::Joy::Ptr&)msg);
            tmp->header.frame_id = "stop";
            interrupt_broadcasting = true;
            pub_joy.publish(tmp);
        }
    }

    //enable motors after bump and/or freerun
    if(msg->buttons[7]) {
        enable_srv.request.enable = true;
        if (!enable_client.call(enable_srv))
        {
            ROS_ERROR("Failed to call service /enable_motors");
        }
        if (!reset_client.call(reset_srv))
        {
            ROS_ERROR("Failed to call service /reset_motorstop");
        }
        if (!barrier_client.call(barrier_srv))
        {
            ROS_ERROR("Failed to call service /reset_barrier_stop");
        }
    }
    //disable motors to move robot manually
    if(msg->buttons[6]) {
        enable_srv.request.enable = false;
        if (!enable_client.call(enable_srv))
        {
            ROS_ERROR("Failed to call service /enable_motors");
        }
    }
    //emergency stop
    if(msg->axes[2] == -1.0) {
        if (!emergency_client.call(emergency_srv))
        {
            ROS_ERROR("Failed to call service /emergency_stop");
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_joystick");

    ros::NodeHandle n("teleop_joystick");
    interrupt_broadcasting = false;
    sent_error = false;

    ros::Subscriber sub = n.subscribe("/joy", 1000, controlCallback);
    pub_joy = n.advertise<sensor_msgs::Joy>("joy", 1000);
    pub_buttons = n.advertise<scitos_teleop::action_buttons>("action_buttons", 1000);
    enable_client = n.serviceClient<scitos_msgs::EnableMotors>("/enable_motors");
    reset_client = n.serviceClient<scitos_msgs::ResetMotorStop>("/reset_motorstop");
    emergency_client = n.serviceClient<scitos_msgs::EmergencyStop>("/emergency_stop");
    barrier_client = n.serviceClient<scitos_msgs::ResetBarrierStop>("/reset_barrier_stop");

    ros::spin();

    return 0;
}
