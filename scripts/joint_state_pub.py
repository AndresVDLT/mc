#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32
import numpy as np
from math import cos, sin

class RobotSimulator:
    def __init__(self):
        
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)


        rospy.Subscriber('/wl', Float32, self.cmd_wl_callback)
        rospy.Subscriber('/wr', Float32, self.cmd_wr_callback)

        self.wl = 0
        self.wl = 0

        self.x = 0
        self.y = 0
        self.psi = 0

        
        self.pose = PoseStamped()
        self.rate = rospy.Rate(10)  # 10Hz

    def cmd_vel_callback(self, msg):
        msg = Twist()
        angular_vel = msg.angular.z
        linear_vel = msg.linear.x
        ra = .05
        b = 0.075
        mat = np.array([ra/2, ra/2], [ra/(2*b), -ra/(2*b)])
        inv_mat = np.linalg.inv(mat)
        input = np.array([linear_vel, angular_vel])
        [vel_r, vel_l] = np.matmul(inv_mat, input)
        self.wl = vel_l
        self.wr = vel_r


    def run(self):
        while not rospy.is_shutdown():
            self.wl +

if __name__ == '__main__':
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass

