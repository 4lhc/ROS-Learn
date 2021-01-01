#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>


void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	//http://docs.ros.org/en/jade/api/tf2_ros/html/c++/classtf2__ros_1_1TransformBroadcaster.html
	static tf2_ros::StaticTransformBroadcaster br; //makes publishing transforms easier
	geometry_msgs::TransformStamped transformStamped;

	transformStamped.header.stamp = ros::Time::now(); //timestamp
	transformStamped.header.frame_id = "odom";
	transformStamped.child_frame_id = "base_footprint";
	transformStamped.transform.translation.x = msg->pose.pose.position.x;
	transformStamped.transform.translation.y = msg->pose.pose.position.y;
	transformStamped.transform.translation.z = 0.0;

	transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
	transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
	transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
	transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

	br.sendTransform(transformStamped);


}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_pkg_static_broadcaster");
	ros::NodeHandle private_node("~");

	ros::NodeHandle node;
	ros::Subscriber sub = node.subscribe("/odom", 10, &odomCallback);
	ros::spin();

	return 0;
}
