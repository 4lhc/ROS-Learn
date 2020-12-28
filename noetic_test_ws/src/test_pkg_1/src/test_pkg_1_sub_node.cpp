#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>

void subCallBack(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO_STREAM("Got message " << msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_pkg_1_subscriber_node");
  ros::NodeHandle nh;

  ros::Subscriber subscriber = nh.subscribe("chatter",
                                               10,							//queue size
                                               subCallBack);
  /*********************************************************************************/
  /* ros::Subscriber subscriber = nh.subscribe("chatter",                          */
  /*                                           10,                                 */
  /*                                           &className::callback_method, this); */
  /*********************************************************************************/

  //rosparams
  std::string rosdistro;
  nh.getParam("/rosdistro", rosdistro);
  ROS_INFO_STREAM_ONCE("ROSDISTRO: " << rosdistro);

  ros::spin();
  return 0;
}
