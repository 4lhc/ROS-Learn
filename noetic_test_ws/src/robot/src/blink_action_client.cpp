#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <robot_msgs/BlinkAction.h>

using BlinkClient = actionlib::SimpleActionClient<robot_msgs::BlinkAction>;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "BlinkClient");
	BlinkClient client("blink", true);
	client.waitForServer();
	robot_msgs::BlinkGoal goal;
	goal.blink_count = 10;
	client.sendGoal(goal);
	client.waitForResult(ros::Duration(10.0));

	if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO_STREAM("SUCCEEDED");
	}

	return 0;
}
