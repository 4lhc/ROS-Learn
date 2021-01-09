#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/timer_options.h>

void timerCallback(const ros::TimerEvent& te)
{
  ROS_INFO_STREAM("Timer callback Triggered" << te.profile.last_duration); 
}


void timerCallback_Q(const ros::TimerEvent& te)
{
  ROS_INFO_STREAM("Timer Callback in CallbackQueue " << te.profile.last_duration);  
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "hw_node_name"); // intializing roscpp node
  ros::NodeHandle nh;
  ros::Rate loopRate(0.1); // 0.1Hz
  ros::Rate loopRate_1(1); // 1Hz

  // This timer will be trigger at the rate of 10Hz even though it is supposed
  // to be called every 1 second (1Hz)
  // https://docs.ros.org/en/api/roscpp/html/structros_1_1TimerOptions.html
  //
  ros::Timer timer = nh.createTimer(ros::Duration(1), timerCallback);

  // Callback queue
  ros::CallbackQueue cb_queue;
  ros::TimerOptions t_opts = ros::TimerOptions(ros::Duration(1),
                                                timerCallback_Q,
                                                &cb_queue);
  ros::Timer timer_cb_q = nh.createTimer(t_opts);

  while(ros::ok())
  {
      ros::spinOnce(); //Single-threaded spinning All callbacks are called
      loopRate.sleep();
      cb_queue.callAvailable();
  }
  return 0;
}
