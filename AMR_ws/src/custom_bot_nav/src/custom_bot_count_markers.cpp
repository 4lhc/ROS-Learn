#include <ros/ros.h>
#include <map>

void markeCallBack(const )

int main(int argc, char** argv)
{
	ros::init(argc, argv, "customBotCountMarkerNode");
	ros::NodeHandle nh;
	ros::Subscriber marker_subscriber = nh.subscribe("/aruco_marker_publisher/markers_list",
																					10,
																					marker_callback);

	std::map<int, bool> marker_checklist{{210, false},
																			 {220, false},
																			 {230, false},
																			 {240, false},
																			 {250, false}};

	for(auto& [key, value] : marker_checklist)
	{
		ROS_INFO_STREAM("marker_" << key << ", status: " << value);
	}
	

	return 0;
}
