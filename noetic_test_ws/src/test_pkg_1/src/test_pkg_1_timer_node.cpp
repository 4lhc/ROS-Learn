#include <ros/ros.h>


void timerCallback(const ros::TimerEvent& te)
{
  ROS_INFO_STREAM("timerCallback triggered");
  ROS_INFO_STREAM(te.last_expected); // When the previous callback should have happened
  ROS_INFO_STREAM(te.last_real); // When the previous callback actually happened
  ROS_INFO_STREAM(te.current_expected); // When the current callback should have been called
  ROS_INFO_STREAM(te.current_real); // When the current callback was actually called
  ROS_INFO_STREAM(te.profile.last_duration); // Duration of the last callbackj
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_node_name"); // intializing roscpp node
  ros::NodeHandle nh;

    ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);




  ros::spin(); // processes incoming messages via callbacks

  return 0;
}
