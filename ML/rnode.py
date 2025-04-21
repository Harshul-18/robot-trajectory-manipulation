#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import json

class ControlTurtlebot:
    def __init__(self):
        rospy.init_node('turtlebot_control', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        self.tf_listener = tf.TransformListener()

        self.position = Point()
        self.yaw = 0
        self.laser_ranges = []

    def odom_callback(self, msg):
        self.position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = tf.transformations.euler_from_quaternion(
            [orientation.x, orientation.y, orientation.z, orientation.w])

    def laser_callback(self, msg):
        self.laser_ranges = list(msg.ranges)

    def move(self, linear_vel, angular_vel):
        twist = Twist()
        twist.linear.x = linear_vel
        twist.angular.z = angular_vel
        self.cmd_vel_pub.publish(twist)

    def get_state(self):
        return {
            'x': self.position.x,
            'y': self.position.y,
            'yaw': self.yaw,
            'laser_ranges': self.laser_ranges
        }

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Write current state to file
            with open('/tmp/turtlebot_state.json', 'w') as f:
                json.dump(self.get_state(), f)

            # Read action from file
            try:
                with open('/tmp/turtlebot_action.json', 'r') as f:
                    action = json.load(f)
                self.move(action['linear'], action['angular'])
            except:
                pass  # No action file or invalid content

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = ControlTurtlebot()
        controller.run()
    except rospy.ROSInterruptException:
        pass
