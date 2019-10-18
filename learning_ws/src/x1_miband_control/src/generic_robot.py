import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Robot:
    def __init__(self, name="robot1", lin_vel=0.3, ang_vel=0.5):
        self.name = name
        print("Robot created")
        rospy.loginfo("Robot {} created".format(self.name))
        self.cmd_vel = Twist()
        self.lin_vel = lin_vel
        self.ang_vel = ang_vel
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.scenario_lookup_table = {  0: self.move_forward,
                                        1: self.turn_left,
                                        2: self.turn_right,
                                        3: self.turn_left,
                                        4: self.turn_right,
                                        5: self.move_forward,
                                        6: self.turn_right,
                                        7: self.stop}
        self.dist_to_obstacle_in_front = 10
        self.dist_to_obstacle_in_left = 10
        self.dist_to_obstacle_in_right = 10



        print("Waiting for connections")
        # while self.pub.get_num_connections() < 1:
            # rospy.loginfo("Waiting for connections")
            # rospy.sleep(0.2)

    def laser_callback(self, msg):
        self.dist_to_obstacle_in_front = min(min(msg.ranges[0:20]), min(msg.ranges[340:359]), 10)
        self.dist_to_obstacle_in_right = min(min(msg.ranges[315:340]), 10)
        self.dist_to_obstacle_in_left = min(min(msg.ranges[20:45]), 10)

    def get_scenario(self):
        ob_lim = 0.5 #obstacle distance
        ob_f = self.dist_to_obstacle_in_front
        ob_l = self.dist_to_obstacle_in_left
        ob_r = self.dist_to_obstacle_in_right
        scenario = [4*(ob_l<ob_lim), 2*(ob_f<ob_lim), 1*(ob_r<ob_lim)]
        case = sum(scenario)
        rospy.loginfo("{} : Case = {} , Scenario = {}".format(self.name,
                                                              case,
                                                              scenario))
        return case

    def obstacle_avoidance(self):
        case = self.get_scenario()
        cmd = self.scenario_lookup_table[case]
        cmd()


    def move(self):
        self.pub.publish(self.cmd_vel)

    def turn_right(self):
        '''cw'''
        rospy.loginfo("Turning right")
        self.cmd_vel.angular.z = -abs(self.ang_vel)
        self.move()

    def turn_left(self):
        '''ccw'''
        rospy.loginfo("Turn Left")
        self.cmd_vel.angular.z = abs(self.ang_vel)
        self.move()

    def move_forward(self):
        rospy.loginfo("Moving forward")
        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = self.lin_vel
        self.move()

    def stop(self):
        rospy.loginfo("Stoping...")
        self.cmd_vel.angular.z = 0
        self.cmd_vel.linear.x = 0
        self.move()

    def set_velocity(self, lin_vel, ang_vel):
        self.cmd_vel.angular.z = ang_vel
        self.cmd_vel.linear.x = lin_vel
        self.move()
