#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <robot_msgs/BlinkAction.h>


class BlinkAction
{
	public:
		BlinkAction(std::string name):
			as_(nh_, name, false),   // Create an action server without executeCB
			action_name_(name)
	{
		//register the goal and feedback callbacks
		as_.registerGoalCallback(boost::bind(&BlinkAction::goalCB, this));
		as_.registerPreemptCallback(boost::bind(&BlinkAction::preemtCB, this));
		// I don't understand boost:bind...
		as_.start();
	}


		~BlinkAction(void)
		{

		}

	void goalCB()
	{
		goal_ = as_.acceptNewGoal()->blink_count;
	}


	void preemptCB()
		{
			// A preemptCB is ncessary to ensure that the action responds promptly to
			// a cancel request
			ROS_INFO_STREAM(action_name << " : Preempted");
		}

}




void blinkCB(const robot_msgs::BlinkGoal::ConstPtr &goal)
{

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "blink_action_server");
	ros::NodeHandle nh;
	auto as = actionlib::SimpleActionServer<robot_msgs::BlinkAction>(nh, "blink", blinkCB);
	// actionlib::SimpleActionServer<robot_msgs::BlinkAction> as;
	return 0;
}
