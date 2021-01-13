#include <actionlib/server/simple_action_server.h>
#include <robot_msgs/BlinkAction.h>


void blinkCB(const robot_msgs::BlinkGoal::ConstPtr &goal)
{

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "blink_action_server");
	ros::NodeHandle nh;
	actionlib::SimpleActionServer<robot_msgs::BlinkAction> as;
	// actionlib::SimpleActionServer<robot_msgs::BlinkAction> as;
	return 0;
}
