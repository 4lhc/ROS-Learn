#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "quaternion_operations");
	//Quternion datatype
	tf2::Quaternion q1;
	//geometry_msgs quaternion datatype
	geometry_msgs::Quaternion gq1;

	q1.setRPY(0, 0, 0); //RPY to Quaternion conversion

	ROS_INFO_STREAM(q1[0] << " "<< q1[1] << " "<< q1[2] << " "<< q1[2]);
	tf2::convert(q1, gq1);
	ROS_INFO_STREAM("no rotation " << gq1);

	q1.normalize();
	ROS_INFO_STREAM(q1); //Doesn't print (0, 0, 0, 1)


	//Rotation
	tf2::Quaternion q_rot, q_new;
	q_rot.setRPY(0, 0, 1.5707963);
	q_new = q_rot*q1;

	ROS_INFO_STREAM(gq1);
	tf2::convert(q_new, gq1);
	ROS_INFO_STREAM(gq1);
	return 0;
}
