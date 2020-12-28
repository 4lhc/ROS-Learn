#include "ros/ros.h"
#include "std_msgs/String.h"

void subCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Got message " << msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pkg_1_subscriber_node");
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe("/test/chatter",
                                               10, 							//queue size
                                               subCallBack);

  ros::spin();
  return 0;
}
