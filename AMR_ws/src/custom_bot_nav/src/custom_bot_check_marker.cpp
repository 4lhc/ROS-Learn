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



#include <ros/ros.h>
#include <map>
#include <std_msgs/UInt32MultiArray.h>
#include <algorithm>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class MarkerCounter{
        public:
                MarkerCounter();
                void markerCallBack(const std_msgs::UInt32MultiArray::ConstPtr& msg);
                void markerCheckTimerCallBack(const ros::TimerEvent& te);

        private:
                //TODO: read from rosparam
                std::map<int, bool> marker_check{{210, false},
                                     {220, false},
                                     {230, false},
                                     {240, false},
                                     {250, false}};
};

MarkerCounter::MarkerCounter()
{

}

void MarkerCounter::markerCallBack(const std_msgs::UInt32MultiArray::ConstPtr& msg)
{

        for(auto m_id : msg->data)
        {
                std::map<int, bool>::iterator it_m = marker_check.find(m_id);
                if (it_m != marker_check.end())
                {
                        ROS_INFO_STREAM_COND(!marker_check[m_id] ,m_id << " checked");
                        marker_check[m_id] = true;
                }
        }
}

void MarkerCounter::markerCheckTimerCallBack(const ros::TimerEvent& te)
{
        // Check periodically, if all markers found

        if (std::all_of(marker_check.begin(), marker_check.end(), [](const auto& e){return e.second;}))
        {
                ROS_INFO_STREAM("All markers found.");
                ros::shutdown();
        }
}


int main(int argc, char** argv)
{
        ros::init(argc, argv, "customBotCountMarkerNode");
        ros::NodeHandle nh;
        MarkerCounter mc = MarkerCounter();
        ros::Subscriber marker_subscriber = nh.subscribe("/aruco_marker_publisher/markers_list",
                                                        10,
                                                        &MarkerCounter::markerCallBack,
                                                        &mc);

        ros::Timer timer = nh.createTimer(ros::Duration(5.0),
                                        &MarkerCounter::markerCheckTimerCallBack,
                                        &mc);
        ROS_INFO_STREAM("Looking for aruco markers");

        ros::spin();


        return 0;
}
