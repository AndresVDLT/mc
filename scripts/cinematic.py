#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32
import numpy as np
from math import cos, sin
from sensor_msgs.msg import JointState

ra = .05
b = 0.191 / 2

class RobotSimulator:
    def __init__(self):
        rospy.init_node('cinematic', anonymous=True)
        
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)


        self.wl_pub = rospy.Publisher('/wl', Float32, queue_size=10)
        self.wr_pub = rospy.Publisher('/wr', Float32, queue_size=10)        

        self.wl = 0
        self.wr = 0


        self.pose = PoseStamped()
        self.rate = rospy.Rate(10)  # 10Hz

    def cmd_vel_callback(self, msg):
        angular_vel = msg.angular.z
        linear_vel = msg.linear.x
        mat = np.array([[ra/2, ra/2], [ra/(2*b), -ra/(2*b)]])
        inv_mat = np.linalg.inv(mat)
        input = np.array([linear_vel, angular_vel])
        resultado = np.matmul(inv_mat, input)
        self.wl = resultado[1]
        self.wr = resultado[0] 


    def run(self):
        x = 0
        y = 0
        psi = 0
        dt = 0.1
        while not rospy.is_shutdown():
            msg = Float32()
            msg.data = self.wl
            self.wl_pub.publish(msg)
            msg.data = self.wr
            self.wr_pub.publish(msg)
            jacobiano = np.array([[ra*cos(psi)/2, ra*cos(psi)/2], [ra*sin(psi)/2, ra*sin(psi)/2], [ra/(2*b), -ra/(2*b)]])
            [x_dot,y_dot,psi_dot] = np.matmul(jacobiano, np.array([self.wl, self.wr]))
            x += x_dot * dt
            y += y_dot * dt
            psi += psi_dot * dt
            self.rate.sleep()
            

if __name__ == '__main__':
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass
