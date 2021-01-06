#include <ros/ros.h>

class Light
{
  public:
    void lightTimerCallback(const ros::TimerEvent& te);

};

void Light::lightTimerCallback(const ros::TimerEvent& te)
{
  ROS_INFO_STREAM("Class Method Timer callback triggered");

}

void wallTimerCallback(const ros::WallTimerEvent& te)
{
  ROS_INFO_STREAM("WallClock " << te.current_real);
}


void timerCallback(const ros::TimerEvent& te)
{
  ROS_INFO_STREAM("timerCallback triggered" << te.current_real);
  // ROS_INFO_STREAM(te.last_expected); // When the previous callback should have happened
  // ROS_INFO_STREAM(te.last_real); // When the previous callback actually happened
  // ROS_INFO_STREAM(te.current_expected); // When the current callback should have been called
  // ROS_INFO_STREAM(te.current_real); // When the current callback was actually called
  // ROS_INFO_STREAM(te.profile.last_duration); // Duration of the last callbackj
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_node_name"); // intializing roscpp node
  ros::NodeHandle nh;


  // Class method callback
  Light light_obj;
  ros::Timer timer2 = nh.createTimer(ros::Duration(4.0), &Light::lightTimerCallback, &light_obj);


  // Function callback
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  // Walltimer -- For the timer to use wall-clock time instead of ROS clock
  ros::WallTimer walltimer = nh.createWallTimer(ros::WallDuration(1), wallTimerCallback);


  ros::spin(); // processes incoming messages via callbacks

  return 0;
}
