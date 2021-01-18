#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  uint32_t shape = visualization_msgs::Marker::CYLINDER;
  // visualization_msgs::Marker::SPHERE
  // visualization_msgs::Marker::ARROW
  // visualization_msgs::Marker::CUBE

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes"; // Namespace
    marker.id = 0;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    marker.lifetime = ros::Duration();

    ROS_INFO_STREAM("Waiting for subscribers");
    while(marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
	{
	  return 0;
	}
      sleep(1);
    }
    marker_pub.publish(marker);
  }
}



