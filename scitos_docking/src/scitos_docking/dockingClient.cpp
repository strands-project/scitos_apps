#include <ros/ros.h>
#include <scitos_docking/ChargingAction.h>
#include <actionlib/client/simple_action_client.h>

void doneCb(const actionlib::SimpleClientGoalState& state,const scitos_docking::ChargingResultConstPtr& result)
{
  printf("Finished in state [%s]\n", state.toString().c_str());
  printf("Answer: %s\n",result->Message.c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  printf("Goal just went active\n");
}

// Called every time feedback is received for the goal
void feedbackCb(const scitos_docking::ChargingFeedbackConstPtr& feedback)
{
  if (feedback->Level == 0) printf("Charging %s, Progress: %i%%\n", feedback->Message.c_str(),feedback->Progress);
  if (feedback->Level == 1) printf("WARNING! Charging reports to be stuck in %i%% of %s\n", feedback->Progress,feedback->Message.c_str());
}
 
 int main (int argc, char **argv)
 {
   ros::init(argc, argv, "test_charging");
 
   actionlib::SimpleActionClient<scitos_docking::ChargingAction> ac("chargingServer", true);
 
   printf("Waiting for action server to start.\n");
   ac.waitForServer();
 
   printf("Charging action server responded, sending goal.\n");
   scitos_docking::ChargingGoal goal;
   goal.Command = argv[1];
   goal.Timeout = atoi(argv[2]);
   usleep(200000);
   ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);
   usleep(200000);
   ros::spin(); 
   printf("Charging client terminating.\n");
   //exit
   return 0;
 }
