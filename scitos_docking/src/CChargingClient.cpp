#include "CChargingClient.h"

void CChargingClient::doneCb(const actionlib::SimpleClientGoalState& state,const scitos_apps_msgs::ChargingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %s",result->Message.c_str());
}

void CChargingClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

void CChargingClient::feedbackCb(const scitos_apps_msgs::ChargingFeedbackConstPtr& feedback)
{
  if (feedback->Level == 0) ROS_INFO("Charging %s, Progress: %i%%", feedback->Message.c_str(),feedback->Progress);
  if (feedback->Level == 1) ROS_INFO("WARNING! Charging reports to be stuck in %i%% of %s", feedback->Progress,feedback->Message.c_str());
}
 
CChargingClient::CChargingClient()
{
}

CChargingClient::~CChargingClient()
{
}

void CChargingClient::initiateCharging()
{
	actionlib::SimpleActionClient<scitos_apps_msgs::ChargingAction> ac("chargingServer", true);
	scitos_apps_msgs::ChargingGoal joyGoal;
	joyGoal.Command = "charge";
	joyGoal.Timeout = 120;
	ac.sendGoal(joyGoal, &doneCb, &activeCb, &feedbackCb);
	ros::spinOnce();
}

void CChargingClient::initiateUndocking()
{
	actionlib::SimpleActionClient<scitos_apps_msgs::ChargingAction> ac("chargingServer", true);
	scitos_apps_msgs::ChargingGoal joyGoal;
	joyGoal.Command = "undock";
	joyGoal.Timeout = 120;
	ac.sendGoal(joyGoal, &doneCb, &activeCb, &feedbackCb);
	ros::spinOnce();
}


