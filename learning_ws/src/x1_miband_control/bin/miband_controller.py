#!/usr/bin/env python
import rospy
from x1_miband_control.MiBand_HRX.base import MiBand2
from x1_miband_control.MiBand_HRX.constants import ALERT_TYPES
from threading import Thread
from generic_robot import Robot


class MiBandController():
    def __init__(self, MAC):
        self.robot = Robot()
        self.band = MiBand2(MAC, debug=True)
        self.band.setSecurityLevel(level="medium")
        if not self.band.authenticate():
            self.band.initialize()
            self.band.authenticate()

        self.t1 = Thread(target=self.start_data_realtime)
        self.prev_signal = (0, 0, 0)

    def accel_call_back(self, x):
        pass

    def start_data_realtime(self):
        self.band.start_raw_data_realtime(self.accel_call_back)

    def start_control(self):
        rospy.loginfo("Starting Control")
        self.t1.start()
        while not self.band.is_realtime_stopped():
            self.vel_control()
            if self.robot.get_scenario() == 7:
                self.band.stop_realtime()
                self.band.send_alert(ALERT_TYPES.MESSAGE)
        self.robot.stop()

    def stop_control(self):
        self.band.stop_realtime()



    def vel_control(self):
        #we will drive lin_vel.x proportional to pitch & ang_vel.x proportional
        #to roll. We don't have yaw information.
        r1, p1, y1 = self.band.get_euler()
        rospy.loginfo("{} {} {}".format(r1, p1, y1))
        r0, p0, y0 = self.prev_signal
        a = 0.5
        #simple low pass filter
        r2, p2, y2 = (a*r1 + (1-a)*r0, a*p1 + (1-a)*p0, 0)

        self.prev_signal = (r2, p2, y2)
        self.robot.set_velocity(lin_vel=p2, ang_vel=-r2)

if __name__ == "__main__":
    rospy.init_node("miband_controller")
    micontrol = MiBandController(MAC="C5:64:8F:FC:47:F2")
    while True:
        try:
            micontrol.start_control()
        except KeyboardInterrupt:
            micontrol.stop_control()



