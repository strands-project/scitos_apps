#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

void doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseActionResultConstPtr& result)
{
//  ROS_INFO("Finished in state [%s]", state.toString().c_str());
//  ROS_INFO("Answer: %s",result->Message.c_str());
  ros::shutdown();
}

// Called once when the goal becomes active
void activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void feedbackCb(const move_base_msgs::MoveBaseActionFeedbackConstPtr& feedback)
{
//  if (feedback->Level == 0) ROS_INFO("Charging %s, Progress: %i%%", feedback->Message.c_str(),feedback->Progress);
//  if (feedback->Level == 1) ROS_INFO("WARNING! Charging reports to be stuck in %i%% of %s", feedback->Progress,feedback->Message.c_str());
}
 
int main (int argc, char **argv)
{
	ros::init(argc, argv, "test_move_charging");

	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	ROS_INFO("Waiting for action server to start.");
	ac.waitForServer();

	//   move_base_msgs::MoveBaseActionGoal goal;
	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();
	if(argc == 5){
		goal.target_pose.pose.position.x = atof(argv[1]);
		goal.target_pose.pose.position.y = atof(argv[2]);
		goal.target_pose.pose.orientation.x = 0;
		goal.target_pose.pose.orientation.y = 0;
		goal.target_pose.pose.orientation.z = atof(argv[3]);
		goal.target_pose.pose.orientation.w = atof(argv[4]);
		printf("A: %f %f",goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
		ac.sendGoal(goal);//, &doneCb, &activeCb, &feedbackCb);
		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			ROS_INFO("Move to waypoint successfull.");
		else
			ROS_INFO("The base failed to reach its destination.");
	}else{
		ac.cancelAllGoals();
	}
	//   goal.Command = argv[1];
	//  goal.Timeout = atoi(argv[2]);


	return 0;
}
