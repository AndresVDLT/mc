#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float32


class RobotSimulator:
    def __init__(self):
        rospy.init_node('robot_simulator', anonymous=True)
        
        self.pose_publisher = rospy.Publisher('/odom', PoseStamped, queue_size=10)


        rospy.Subscriber('/wl', Float32, self.cmd_wl_callback)
        rospy.Subscriber('/wr', Float32, self.cmd_wr_callback)
        
        self.pose = PoseStamped()
        
        self.rate = rospy.Rate(10)  # 10Hz

    def cmd_vel_callback(self, msg):
        # Implement the logic to control the robot based on Twist message
        # For simplicity, let's just update the robot's pose based on linear and angular velocities
        # Here, you would typically implement your robot's kinematics or dynamics model
        # For this example, we'll just update the position based on velocity
        self.pose.pose.position.x += msg.linear.x / 10.0  # Divide by 10.0 to scale the velocity
        self.pose.pose.position.y += msg.linear.y / 10.0
        self.pose.pose.s

        self.pose.pose.orientation.w += msg.angular.z / 10.0  # Divide by 10.0 to scale the angular velocity

    def run(self):
        while not rospy.is_shutdown():
            self.pose.header.stamp = rospy.Time.now()
            self.pose_publisher.publish(self.pose)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass
