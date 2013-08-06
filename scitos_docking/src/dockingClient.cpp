#include <ros/ros.h>
#include <scitos_apps_msgs/ChargingAction.h>
#include <actionlib/client/simple_action_client.h>

void doneCb(const actionlib::SimpleClientGoalState& state,const scitos_apps_msgs::ChargingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %s",result->Message.c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const scitos_apps_msgs::ChargingFeedbackConstPtr& feedback)
{
  if (feedback->Level == 0) ROS_INFO("Charging %s, Progress: %i%%", feedback->Message.c_str(),feedback->Progress);
  if (feedback->Level == 1) ROS_INFO("WARNING! Charging reports to be stuck in %i%% of %s", feedback->Progress,feedback->Message.c_str());
}
 
 int main (int argc, char **argv)
 {
   ros::init(argc, argv, "test_charging");
 
   actionlib::SimpleActionClient<scitos_apps_msgs::ChargingAction> ac("chargingServer", true);
 
   ROS_INFO("Waiting for action server to start.");
   ac.waitForServer();
 
   ROS_INFO("Action server started, sending goal.");
   scitos_apps_msgs::ChargingGoal goal;
   goal.Command = argv[1];
   goal.Timeout = atoi(argv[2]);
   ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
   ros::spin(); 
   //exit
   return 0;
 }
