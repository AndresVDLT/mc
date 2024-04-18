#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class RobotSimulator:
    def __init__(self):
        rospy.init_node('robot_simulator', anonymous=True)
        
        self.pose_publisher = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Suscribe to the robot's motor nodes
        rospy.Subscriber('/wl', Float32, self.cmd_wl_callback)
        rospy.Subscriber('/wr', Float32, self.cmd_wr_callback)
        
        self.pose = Odometry()
        self.rate = rospy.Rate(10)  # 10Hz
        

    def cmd_wr_callback(self, msg):
        # Save the message in the variable named wr
        self.wr = msg.data

    def cmd_wl_callback(self, msg):
        # Save the message in the variable named wl
        self.wl = msg.data

    def run(self):
        x = 0
        y = 0
        o = 0
        while not rospy.is_shutdown():
            # Update the timestamp of the pose
            self.pose.header.stamp = rospy.Time.now()
            dt = 0.1
            self.matA = np.array([
                                [0.025, 0.025],
                                [0.2632, -0.2632]
                                ])
            self.arr = np.array([self.wr, self.wl])

            Vw = np.matmul(self.matA, self.arr)
            y_dot =  Vw[0] * sin(o)
            y +=  y_dot * dt
            x_dot =  Vw[0] * cos(o)
            x += x_dot * dt
            o += Vw[1] * dt

            #Creamos el msg
            current_time = rospy.Time.now()
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "odom"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.orientation = transformations.quaternion_from_euler(o)
            odom.child_frame_id = "base_link"
            odom.twist.twist.angular.z = Vw[1]
            odom.twist.twist.linear.x = x_dot
            odom.twist.twist.linear.y = y_dot
            # Publish the pose
            self.pose_publisher.publish(self.pose)
            # Sleep to maintain the specified rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot_sim = RobotSimulator()
        robot_sim.run()
    except rospy.ROSInterruptException:
        pass
