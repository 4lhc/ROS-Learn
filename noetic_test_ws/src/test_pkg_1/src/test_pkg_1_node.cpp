#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_node_name"); // intializing roscpp node
  ros::NodeHandle nh;

  /**********************************************************************/
  /* nh_ = ros::NodeHandle();  // /namespace/ropic                      */
  /* nh_pvt_ = ros::NodeHandle("~");  // /namespace/node/topic          */
  /* nh_omni_ = ros::NodeHandle("omnibot"); // /namespace/omnibot/topic */
  /* nh_global_= ros::NodeHangle("/"); // /topic :not recommended       */
  /**********************************************************************/

  ros::Rate loopRate(10); // 10Hz

      unsigned int count = 0;
  while (ros::ok())
  {
    ROS_INFO_STREAM("First roscpp node" << count);
    ROS_DEBUG_STREAM("First roscpp debug" << count);
    ROS_WARN_STREAM_COND(count > 10,
                         "count greater than 10, count = " << count);
    ROS_ERROR_STREAM_ONCE("Print only once");

    ros::spinOnce(); // processes incoming messages via callbacks
    loopRate.sleep();
    count++;
  }
  return 0;
}
