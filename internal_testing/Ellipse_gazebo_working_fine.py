#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import numpy as np
import heapq
from simple_pid import PID
import time
import os
import matplotlib.pyplot as plt

class ControlTurtlebot():
    def __init__(self, initial_position, kp, ki, kd, i, a, b, num_points=12, dt=0.1):
        rospy.init_node('turtlebot3_control_node', anonymous=True)
        rospy.loginfo('To stop the turtlebot, press ctrl+c')
        
        self.is_shutdown = False
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback)

        self.tf_listener = tf.TransformListener()
        self.odom_frame = 'odom'

        try:
            self.tf_listener.waitForTransform(self.odom_frame, 'base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = 'base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, 'base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = 'base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between odom and base_link or base_footprint")
                rospy.signal_shutdown("tf Exception")

        (position, rotation) = self.get_odom()

        self.vel_msg = Twist()
        
        self.ys = []
        self.xs = []

        # print('Initial Position: ', 'Pixel', initial_pixel, 'Gazebo', initial_position)
        # print('Final Position: ', 'Pixel', final_pixel, 'Gazebo', final_position)

        self.a = a
        self.b = b
        self.num_points = num_points
        self.path = self.calculate_path(self.a, self.b, self.num_points)
        # print(self.path)

        self.initial_position = initial_position

        # self.path = []
        print(self.path)
        # return 

        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.filename = i

        self.dt = dt

        rospy.sleep(2)

    def calculate_path(self, a, b, num_points = 12):
        theta = np.linspace(0, 2 * np.pi, num_points)
        x = float(a) * np.cos(theta)
        y = float(b) * np.sin(theta)
        waypoints = list(zip(x, y))
        return waypoints
    

    def get_odom(self):
        try:
            (trans, rot) = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
            rotation = euler_from_quaternion(rot)
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), rotation[2])

    def get_distance_error(self, x, y, x_d, y_d):
        e_x = x_d - x
        e_y = y_d - y
        return np.sqrt(e_x**2 + e_y**2)

    def get_heading_error(self, x, y, theta, x_d, y_d):
        desired_heading = math.atan2(y_d - y, x_d - x)
        heading_error = theta - desired_heading
        return math.atan2(np.sin(heading_error), np.cos(heading_error))

    def move_along_path(self):
        pid_distance = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0)
        pid_heading = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0)

        pid_distance.output_limits = (0.1, 1.0)
        # pid_heading.output_limits = (-1.0, 1.0)

        i = 0

        for waypoint in self.path:
            x_d, y_d = waypoint
            print(x_d, y_d)

            while True:
                (position, rotation) = self.get_odom()
                e_theta = self.get_heading_error(position.x, position.y, rotation, x_d, y_d)

                if self.is_shutdown:
                    break

                if abs(e_theta) < 0.1:
                    msg = Twist()
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.cmd_vel.publish(msg)
                    break

                msg = Twist()
                msg.angular.z = 0.5 * (-abs(e_theta)/e_theta)
                self.cmd_vel.publish(msg)

                rospy.sleep(self.dt)

            # print('first loop cleared')
            rospy.sleep(1)

            while True:
                (position, rotation) = self.get_odom()
                self.xs.append(position.x)
                self.ys.append(position.y)
                e_d = self.get_distance_error(position.x, position.y, x_d, y_d)
                e_theta = self.get_heading_error(position.x, position.y, rotation, x_d, y_d)

                # print(e_d, e_theta)

                pid_distance.setpoint = 0
                pid_heading.setpoint = 0

                linear_velocity = pid_distance(e_d)
                angular_velocity = pid_heading(e_theta)

                # print("linear_velocity: {}; angular_velocity: {}; real_pos: {}; real_theta: {}".format(
                #     linear_velocity,
                #     angular_velocity,
                #     position,
                #     rotation
                # ))

                # if self.msg is not None:
                #     print('\n')
                #     print('0:\t', str(self.msg[0]))
                #     print('90:\t', str(self.msg[90]))
                #     print('180:\t', str(self.msg[180]))
                #     print('270:\t', str(self.msg[270]))
                #     print('359:\t', str(self.msg[359]))

                #     if min(self.msg[-45:] + self.msg[:45]) < 0.5:
                #         linear_velocity = 0
                #         angular_velocity = 0.5

                # self.ys.append(position.y)
                # self.xs.append(position.x)

                if self.is_shutdown:
                    break

                if e_d < 0.05:
                    msg = Twist()
                    msg.linear.x = 0
                    msg.angular.z = 0
                    self.cmd_vel.publish(msg)
                    break

                msg = Twist()
                msg.linear.x = linear_velocity
                msg.angular.z = angular_velocity
                self.cmd_vel.publish(msg)

                rospy.sleep(self.dt)

                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.cmd_vel.publish(msg)

                end = time.time()

            if i == 0:
                self.xs = []
                self.ys = []
                i += 1

        rospy.on_shutdown(self.shutdown)
        return self.xs, self.ys


    def callback(self, msg):
        return msg

    def shutdown(self):
        rospy.loginfo('Stopping the turtlebot')
        shutdown_msg = Twist()
        shutdown_msg.linear.x = 0
        shutdown_msg.angular.z = 0
        self.cmd_vel.publish(shutdown_msg)
        self.is_shutdown = True
        rospy.sleep(1)

if __name__ == "__main__":
    try:
        initial_position = [0, 0] 
        q = quaternion_from_euler(0, 0, 0)
        # state_msg is an object
        state_msg = ModelState()
        state_msg.model_name = 'turtlebot3_burger'
        state_msg.pose.position.x = initial_position[0]
        state_msg.pose.position.y = initial_position[1]
        state_msg.pose.position.z = 0

        state_msg.pose.orientation.x = q[0]
        state_msg.pose.orientation.y = q[1]
        state_msg.pose.orientation.z = q[2]
        state_msg.pose.orientation.w = q[3]

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

        kp, ki, kd = 1, 0.1, 0.05
        i = 'Result'

        a, b = 0.5, 0.25
        num_points = 9

        controller = ControlTurtlebot(initial_position, kp, ki, kd, i, a, b, num_points)
        xs, ys = controller.move_along_path()

        points_given_to_controller = controller.calculate_path(a, b, num_points=num_points)
        xp = [i[0] for i in points_given_to_controller]
        yp = [i[1] for i in points_given_to_controller]

        theoretical_path = controller.calculate_path(a, b, num_points=len(xs))
        xt = [i[0] for i in theoretical_path]
        yt = [i[1] for i in theoretical_path]

        theoretical_path = list(zip(xt, yt))
        practical_path = list(zip(xs, ys))

        error = 0
        for i in range(0, len(theoretical_path)):
            error += math.sqrt((xs[i] - xt[i])**2 + (ys[i] - yt[i])**2)

        print(len(xt), len(xs))
        print(xs[0], ys[0])
        print(xt[0], yt[0])
        plt.plot(xs, ys, color='blue', label='Real Bot Trajectory')
        plt.plot(xt, yt, color='red', label='Theoretical Trajectory')
        plt.scatter(xp, yp, color='black', marker='o', label='Trajectory Points Given')
        plt.title("Ellipse Generation (error = {}, No. of Points = {})".format(round(error, 2), num_points))
        plt.xlabel('x')
        plt.ylabel('y')
        plt.legend()
        plt.grid(True)
        plt.savefig('Ellipse.png')
        plt.show()
    except rospy.ROSInterruptException:
        rospy.loginfo("ControlTurtlebot node terminated.")