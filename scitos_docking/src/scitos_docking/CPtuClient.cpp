#include "scitos_docking/CPtuClient.h"

void CPtuClient::doneCb(const actionlib::SimpleClientGoalState& state,const scitos_docking::ChargingResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  ROS_INFO("Answer: %s",result->Message.c_str());
}

void CPtuClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

void CPtuClient::feedbackCb(const scitos_docking::ChargingFeedbackConstPtr& feedback)
{
  if (feedback->Level == 0) ROS_INFO("Charging %s, Progress: %i%%", feedback->Message.c_str(),feedback->Progress);
  if (feedback->Level == 1) ROS_INFO("WARNING! Charging reports to be stuck in %i%% of %s", feedback->Progress,feedback->Message.c_str());
}
 
CPtuClient::CPtuClient()
{
}

CPtuClient::~CPtuClient()
{
}

