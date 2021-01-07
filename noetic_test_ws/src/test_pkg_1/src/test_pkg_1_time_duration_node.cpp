#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_pkg_1_time_duration_node"); // intializing roscpp node
  ros::NodeHandle nh;
  ros::Rate loopRate(10); // 10Hz

  nh.setParam("/use_sim_time", true);
  // When using simulated Clock time, now() returns time 0 until first message
  // has been received on /clock, so 0 means essentially that the client does
  // not know clock time yet. A value of 0 should therefore be treated
  // differently, such as looping over now() until non-zero is returned.

  // while(ros::ok)
  // {
  //   ROS_INFO_STREAM("Time now: " << ros::Time::now());
  //   ros::spinOnce();
  //   loopRate.sleep();
  // }

  //Difference between Duration::sleep() and Rate::sleep
  ros::Duration(1).sleep(); //Sleep for one second

  while(ros::ok())
  {
    //
    //Some stuff done here
    //
    loopRate.sleep(); // the Rate instance will attempt to keep the loop at
                      // 10hz by accounting for the time used by the work done
                      // during the loop
    // Timers are recommended over Rate
  }

  //Use WallTime, WallDuration and WallRate when wall-clock is needed


  return 0;
}
