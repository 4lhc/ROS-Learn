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

	while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/my_frame";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes"; // Namespace
    marker.id = 0;

		marker.type = shape;


