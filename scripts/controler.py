#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import atan2, sqrt



class talker():
    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('talker', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.set_x = 10
        self.set_y = 10
        self.x = 0
        self.y = 0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.th = msg.pose.pose.orientation.z

    def run(self):
        while not rospy.is_shutdown():
            ey = self.set_y - self.y
            ex = self.set_x - self.x
            ang = atan2(ey, ex)
            ed = sqrt(ex * ex + ey*ey)
            eth = self.th - ang
            vel_msg = Twist()
            vel_msg.linear.x =  ed * 0.1# Change this value to what you want
            vel_msg.angular.z = eth * 0.1 # Change this value to what you want
            rospy.loginfo(vel_msg)
            self.pub.publish(vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass