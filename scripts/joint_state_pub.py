#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
import tf2_ros


class RobotSimulator:
    def __init__(self):
        
        
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        

        self.odomMsg = Odometry()
        joint_state = JointState()
        joint_state.name = ['wheel_coupler_joint', 'wheel_coupler_joint_2']
        joint_state.effort = []
        joint_state.position = [0, 0]  
        self.jointState = joint_state

    def odom_callback(self, msg):
        # Update joint states
        self.joint_state.header = msg.header  
        self.joint_state.velocity = [msg.twist.twist.linear.x, msg.twist.twist.angular.z]
        # Publish joint states
        self.pub.publish(self.joint_state)
        # Publish transform using tf2 (from 'odom' to 'base_link')
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
         # Transform broadcaster
        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(t) 


    def run(self):
        rospy.spin()
            

if __name__ == '__main__':
    rospy.init_node("joint_state_pub")
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass
