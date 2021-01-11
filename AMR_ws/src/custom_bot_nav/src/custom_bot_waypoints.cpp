

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;

class WayPoints{
        public:
                WayPoints();
                void addWaypoint();
        private:
                //TODO: read from rosparam
                std::vector<Pose> waypoints;
};

WayPoints::WayPoints()
{

}

void WayPoints::addWaypoint()
{
        Pose robot_pose = Pose();
        robot_pose.position = p;
        robot_pose.orientation = q;
        waypoints.emplace_back(robot_pose);
}


int main(int argc, char** argv)
{
        ros::init(argc, argv, "customBotWaypoints");
        ros::NodeHandle nh;
        // ros::Subscriber marker_subscriber = nh.subscribe("/aruco_marker_publisher/markers_list",
        //                                                 10,
        //                                                 &MarkerCounter::markerCallBack,
        //                                                 &mc);
        // 
        // ros::Timer timer = nh.createTimer(ros::Duration(5.0),
        //                                 &MarkerCounter::markerCheckTimerCallBack,
        //                                 &mc);
        //

/*
 *      waypoints
 *      frame_id: "odom"
 *      2, 0, 0, 0, 0, 0, 1
 *      1.75, -6, 0, 0, 0, -0.9, 0.4
 *      3.8, -7.6, 0, 0, 0, -0.3, 0.9
 *      6.7, -9.4, 0, 0, 0, 0.9, 0.3
 *      3.7, -10, 0.0, 0., 0, 0.88, 0.4
 *
 *
 */

        WayPoints wp = WayPoints();

        Point p1 = Point()

        ROS_INFO_STREAM("Looking for aruco markers");

        ros::spin();


        return 0;
}
