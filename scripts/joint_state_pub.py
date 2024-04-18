#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from std_msgs.msg import Float32
import numpy as np
from math import cos, sin
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry


ra = .05
b = 0.075

class RobotSimulator:
    def __init__(self):
        
        
        rospy.Subscriber('/cmd_vel', Twist, self.odom_callback)
        rospy.Subscriber('/odom', Odometry, self.cmd_vel_callback)


        self.wl_pub = rospy.Publisher('/wl', Float32, queue_size=10)
        self.wr_pub = rospy.Publisher('/wr', Float32, queue_size=10)

        self.wl_pub2 = rospy.Publisher('/joint_states', JointState, queue_size=10)
        

        self.wl = 0
        self.wl = 0


        
        self.pose = PoseStamped()
        self.rate = rospy.Rate(10)  # 10Hz

    def odom_callback(self, msg):
        msg = Twist()
        angular_vel = msg.angular.z
        linear_vel = msg.linear.x
        mat = np.array([ra/2, ra/2], [ra/(2*b), -ra/(2*b)])
        inv_mat = np.linalg.inv(mat)
        input = np.array([linear_vel, angular_vel])
        [vel_r, vel_l] = np.matmul(inv_mat, input)
        self.wl = vel_l
        self.wr = vel_r
chassis_joint
    def odom(self, msg):
        msg = Twist()
        angular_vel = msg.angular.z
        linear_vel = msg.linear.x
        mat = np.array([ra/2, ra/2], [ra/(2*b), -ra/(2*b)])
        inv_mat = np.linalg.inv(mat)
        input = np.array([linear_vel, angular_vel])
        [vel_r, vel_l] = np.matmul(inv_mat, input)
        self.wl = vel_l
        self.wr = vel_r


    def run(self):
        pub_msg = JointState()
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.velocity = [1]
        pub_msg.position = (1,1)
        pub_msg.name = ["chassis"]
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
            jacobiano = np.array([ra*cos(psi)/2, ra*cos(psi)/2], [ra*sin(psi)/2, ra*sin(psi)/2], [ra/(2*b), -ra/(2*b)])
            [x_dot,y_dot,psi_dot] = np.matmul(jacobiano, np.array([self.wl, self.wr]))
            x += x_dot * dt
            y += y_dot * dt
            psi += psi_dot * dt
            self.rate.sleep()
            

if __name__ == '__main__':
    rospy.init_node("joint_state_pub")
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass
