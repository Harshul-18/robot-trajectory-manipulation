#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
import matplotlib.pyplot as plt
from simple_pid import PID
import time
import numpy as np
from geometry_msgs.msg import Twist, Point, Quaternion
import os
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

errors = []

class ControlTurtlebot():
    def __init__(self, initial_position, theta, final_position, kp, ki, kd, i, dt=0.1):
        rospy.init_node('turtlebot3_control_node', anonymous=True)
        rospy.loginfo('To stop the turtlebot, press ctrl+c')
        
        self.is_shutdown = False
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.callback)

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

        # print(rotation)
        # return

        self.vel_msg = Twist()
        self.msg = None
        
        self.ys = []
        self.xs = []

        self.x_d, self.y_d = final_position
        self.x, self.y = position.x, position.y
        self.theta = rotation
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.filename = i

        self.dt = dt

        error = self.pid(self.kp, self.ki, self.kd, self.x_d, self.y_d, self.x, self.y, self.theta, self.dt)
        if error != 0:
            errors.append(error)

        rospy.sleep(2)

        return

        # rospy.on_shutdown(self.shutdown)
        # rospy.spin()
        # rospy.on_shutdown(self.shutdown)

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
        return math.sqrt(e_x**2 + e_y**2)

    def get_heading_error(self, x, y, theta, x_d, y_d):
        desired_heading = math.atan2(y_d - y, x_d - x)
        heading_error = theta - desired_heading
        return math.atan2(math.sin(heading_error), math.cos(heading_error))

    def pid(self, kp, ki, kd, x_d, y_d, x, y, theta, dt):
        pid_distance = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0)
        pid_heading = PID(Kp=kp, Ki=ki, Kd=kd, setpoint=0)

        pid_distance.output_limits = (0.1, 1.0)
        # pid_heading.output_limits = (-1.0, 1.0)

        start = time.time()

        while True:
            (position, rotation) = self.get_odom()
            e_theta = self.get_heading_error(position.x, position.y, rotation, x_d, y_d)

            if self.is_shutdown:
                break

            # print(e_theta, rotation)

            if abs(e_theta) < 0.01:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.cmd_vel.publish(msg)
                break

            msg = Twist()
            msg.angular.z = 0.05 * (-abs(e_theta)/e_theta)
            self.cmd_vel.publish(msg)


            rospy.sleep(dt)

            # msg = Twist()
            # msg.angular.z = 0
            # self.cmd_vel.publish(msg)

        print(x_d, y_d)

        rospy.sleep(1)


        while True:
            (position, rotation) = self.get_odom()
            e_d = self.get_distance_error(position.x, position.y, x_d, y_d)
            e_theta = self.get_heading_error(position.x, position.y, rotation, x_d, y_d)

            # print(e_d, e_theta)

            pid_distance.setpoint = 0
            pid_heading.setpoint = 0

            linear_velocity = pid_distance(e_d)
            angular_velocity = pid_heading(e_theta)

            # x += linear_velocity * math.cos(theta) * dt
            # y += linear_velocity * math.sin(theta) * dt
            # theta += angular_velocity * dt
            # theta = (theta + math.pi) % (2*math.pi) - math.pi

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

            self.ys.append(position.y)
            self.xs.append(position.x)

            if self.is_shutdown:
                break

            if e_d < 0.06:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.cmd_vel.publish(msg)
                break

            msg = Twist()
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity
            self.cmd_vel.publish(msg)

            rospy.sleep(dt)

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self.cmd_vel.publish(msg)

            end = time.time()
            # if end - start >= 20:
            #     break

            # rospy.sleep(3)

        
        x_values = self.xs
        m = y_d/x_d
        y_values = [m*x for x in self.xs]

        fig, axs = plt.subplots(3, 1, figsize=(5, 15))

        axs[0].plot(self.xs, self.ys, label='PID Controlled Trajectory')
        axs[0].scatter(x_values, y_values, c='r', marker='.', label='Straight Path')

        axs[1].plot(self.xs, self.ys, label='PID Controlled Trajectory')
        axs[1].plot(x_values, y_values, label='Straight Path')
        axs[1].fill_between(self.xs, self.ys, y_values, where=(y_values > self.ys), interpolate=True, color='lightcoral', alpha=0.5)
        axs[1].fill_between(self.xs, self.ys, y_values, where=(y_values <= self.ys), interpolate=True, color='lightcoral', alpha=0.5)

        axs[2].plot(self.xs, self.ys, label='PID Controlled Trajectory')
        axs[2].plot(x_values, y_values, label='Straight Path')
        axs[2].fill_between(self.xs, self.ys, y_values, where=(y_values > self.ys), interpolate=True, color='lightcoral', alpha=0.5)
        axs[2].fill_between(self.xs, self.ys, y_values, where=(y_values <= self.ys), interpolate=True, color='lightcoral', alpha=0.5)
        for i in range(len(self.xs)):
            axs[2].plot([self.xs[i], self.xs[i]], [self.ys[i], y_values[i]], color='black')

        difference = np.abs(np.array(self.ys) - np.array(y_values))
        area = np.trapz(difference, self.xs)

        # if end - start >= 30:
        #     area = float('inf')
        # print(area)

        titles = ['Trajectory Plot', 'Area between the curves', 'Applying Trapeziums Area Calculation']
        for i in range(3):
            axs[i].set_title(titles[i])
            axs[i].set_xlabel('x')
            axs[i].set_ylabel('y')
            axs[i].set_xlim([0, 1.4])
            axs[i].set_ylim([0, 1.4])
            axs[i].legend()
            axs[i].grid(True)
        print(axs.shape)
        axs[0].add_patch(plt.Circle((1, 1), 0.06, color='r', alpha=0.2))
        axs[1].add_patch(plt.Circle((1, 1), 0.06, color='r', alpha=0.2))
        axs[2].add_patch(plt.Circle((1, 1), 0.06, color='r', alpha=0.2))

        fig.suptitle('Trajectory Controlling using PID (Error = {:.3f})'.format(area), fontsize=14)
        plt.tight_layout(rect=[0, 0, 1, 0.9])

        if not os.path.exists('./plot_results'):
            os.mkdir('./plot_results')
        plt.savefig('./plot_results/{}.png'.format(self.filename))

        # plt.show()
        return area

        rospy.on_shutdown(self.shutdown)

    def callback(self, msg):
        self.msg = msg.ranges
    
    def shutdown(self):
        rospy.loginfo('Stopping the turtlebot')
        shutdown_msg = Twist()
        shutdown_msg.linear.x = 0
        shutdown_msg.angular.z = 0
        self.cmd_vel.publish(shutdown_msg)
        self.is_shutdown = True
        rospy.sleep(1)


data = [
    # kp, ki, kd, final_x, final_y
    [1, 0.1, 0.05, 1, 1, 'Testing Real Bot'],
]

results = {
    'No.': [],
    'kp': [],
    'ki': [],
    'kd': [],
    'final_x': [],
    'final_y': [],
    'error': [],
    'time': [],
}

if __name__ == "__main__":
    try:
        i = 0
        for kp, ki, kd, final_x, final_y, filename in data:
            start = time.time()
            error = ControlTurtlebot([0, 0], 0, [final_x, final_y], kp=kp, kd=kd, ki=ki, i=filename)
            end = time.time()

            i += 1
            # if end - start > 20:
            #     continue

            results['No.'].append(i)
            results['kp'].append(kp)
            results['kd'].append(kd)
            results['ki'].append(ki)
            results['final_x'].append(final_x)
            results['final_y'].append(final_y)
            results['time'].append(end-start)

            rospy.sleep(1)
            
        results['error'] = errors
        import pandas as pd
        pd.DataFrame.from_dict(results).to_csv('./plot_results/pid_results.csv')
    except rospy.ROSInterruptException:
        rospy.loginfo("ControlTurtlebot node terminated.")
