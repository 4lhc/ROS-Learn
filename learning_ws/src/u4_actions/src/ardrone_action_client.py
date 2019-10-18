#!/usr/bin/env python
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

nImage = 1

cmd = Twist()
rospy.init_node('drone_action_client')

def takeoff():
    global cmd
    rospy.loginfo("Takeoff - waiting for connections")
    while takeoff_pub.get_num_connections() < 1:
        rospy.sleep(1)
    takeoff_pub.publish()
    cmd.linear.y = 1
    cmd.angular.z = 1
    rospy.loginfo("Takeoff - Done")

def land():
    land_pub.publish()
    rospy.loginfo("Landing - Done")

def fly_drone():
    global cmd
    rospy.loginfo("Fly Drone...")
    cmd.linear.y = -cmd.linear.y
    cmd.angular.z = -cmd.angular.z
    cmd_vel_pub.publish(cmd)

def stop_drone():
    global cmd
    rospy.loginfo("Stop Drone...")
    cmd = Twist()
    cmd_vel_pub.publish(cmd)

###### feedback ######
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1


client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
client.wait_for_server()

takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

takeoff()
###### goal ######
goal = ArdroneGoal(nseconds=10) #take picture for 10 seconds
client.send_goal(goal, feedback_cb=feedback_callback)

# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

fly_drone()
while not client.wait_for_result():
    rospy.loginfo("State : {}".format(client.get_state()))
    rospy.sleep(3)
stop_drone()
land()


###### status ######
# client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
