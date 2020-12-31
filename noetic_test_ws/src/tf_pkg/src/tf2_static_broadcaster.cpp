#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>


int main(int argc, char** argv)
{
	ros::init(argc, argv, "tf_pkg_static_broadcaster");
	return 0;
}
