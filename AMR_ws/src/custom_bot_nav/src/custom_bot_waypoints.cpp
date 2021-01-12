

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <vector>
#include <algorithm>
#include <sstream>

using geometry_msgs::Pose;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;


int main(int argc, char** argv)
{
        int num_of_waypoints{0};
        // int waypoint_id{0};
        std::vector<double> w_position(3, 0.0);
        std::vector<double> w_quaternion(4, 0.0);
        std::stringstream ss;

        ros::init(argc, argv, "customBotWaypoints");
        ros::NodeHandle nh;
        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(2.0)))
        {
                ROS_INFO_STREAM("Waiting for move_base action server");
        }

        if (nh.hasParam("/waypoints/num_of_waypoints"))
        {
                nh.getParam("/waypoints/num_of_waypoints", num_of_waypoints);
                for (int i=0; i < num_of_waypoints; i++)
                {
                        // ss.str("")
                        // ss << "/waypoints/waypoint" << i << "/id";
                        // nh.getParam(ss.str(), waypoint_id);
                        ss.str("");
                        ss << "/waypoints/waypoint" << i << "/position";
                        nh.getParam(ss.str(), w_position);
                        ss.str("");
                        ss << "/waypoints/waypoint" << i << "/orientation";
                        nh.getParam(ss.str(), w_quaternion);

                        Point p = Point();
                        p.x = w_position[0];
                        p.y = w_position[1];
                        p.z = w_position[2];
                        Quaternion q = Quaternion();
                        q.x = w_quaternion[0];
                        q.y = w_quaternion[1];
                        q.z = w_quaternion[2];
                        q.w = w_quaternion[3];

                        ROS_INFO_STREAM("Waypoint" << i << "; Pose " << p << q);

                        move_base_msgs::MoveBaseGoal goal;
                        goal.target_pose.header.frame_id = "odom";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose.position = p;
                        goal.target_pose.pose.orientation = q;

                        ac.sendGoal(goal);
                        ac.waitForResult();

                        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                                ROS_INFO_STREAM("WayPoint" << i << " suceeded.");
                        else
                                ROS_WARN_STREAM("WayPoint" << i << " failed.");
                }
        }
        else
        {
                ROS_ERROR_STREAM("Unable to process /waypoints/num_of_waypoints");
                ros::shutdown();
        }


        ROS_INFO_STREAM("Shutting down waypoint node");
        ros::shutdown();


        return 0;
}
