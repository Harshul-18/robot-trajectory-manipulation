#!/usr/bin/env python

import time
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import LaserScan
import tf
import os
from scipy.special import gamma

class NonLinearFractionalPID:
    def __init__(self, Kp, Ki, Kd, alpha, setpoint=0, output_limits=(None, None)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.alpha = alpha
        self.setpoint = setpoint
        self.output_limits = output_limits
        self._last_time = None
        self._last_error = None
        self._integral = 0
        self._errors = []
        self._times = []

    def __call__(self, current_value):
        error = self.setpoint - current_value
        current_time = time.time()

        if self._last_time is None:
            delta_time = 0
        else:
            delta_time = current_time - self._last_time

        self._errors.append(error)
        self._times.append(delta_time)

        integral = self.calculate_fractional_integral(self._errors, self._times, self.alpha)
        derivative = self.calculate_fractional_derivative(self._errors, self._times, self.alpha)

        output = self.Kp * self.non_linear_function(error) + self.Ki * integral + self.Kd * derivative

        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)

        self._last_time = current_time
        self._last_error = error

        return output

    def calculate_fractional_integral(self, errors, times, alpha):
        integral = 0
        for i in range(len(errors)):
            t_diff = sum(times[:i+1])
            if t_diff > 0:
                integral += errors[i] * (t_diff ** (alpha - 1)) / gamma(alpha)
        return integral

    def calculate_fractional_derivative(self, errors, times, alpha):
        derivative = 0
        for i in range(1, len(errors)):
            t_diff = times[i]
            if t_diff > 0:
                derivative += (errors[i] - errors[i-1]) / (t_diff ** (1 - alpha)) / gamma(2 - alpha)
        return derivative

    def non_linear_function(self, error):
        # Example non-linear function: sigmoid function
        return 1 / (1 + math.exp(-error))

class ControlTurtlebot():
    def __init__(self, initial_position, theta, final_position, kp, ki, kd, alpha, i, dt=0.1):
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

        self.vel_msg = Twist()
        self.msg = None
        
        self.ys = []
        self.xs = []
	self.ts = []

        self.x_d, self.y_d = final_position
        self.x, self.y = position.x, position.y
        self.theta = rotation
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.alpha = alpha
        self.filename = i

        self.dt = dt

        self.pid_distance = NonLinearFractionalPID(Kp=self.kp, Ki=self.ki, Kd=self.kd, alpha=self.alpha, setpoint=0)
        self.pid_heading = NonLinearFractionalPID(Kp=self.kp, Ki=self.ki, Kd=self.kd, alpha=self.alpha, setpoint=0)
        self.pid_distance.output_limits = (0.1, 1.0)

        self.run()

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

    def run(self):
        while True:
            (position, rotation) = self.get_odom()
            e_theta = self.get_heading_error(position.x, position.y, rotation, self.x_d, self.y_d)

            if self.is_shutdown:
                break

            if abs(e_theta) < 0.02:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.cmd_vel.publish(msg)
                break

            msg = Twist()
            msg.angular.z = 0.5 * (-abs(e_theta)/e_theta)
            self.cmd_vel.publish(msg)

            rospy.sleep(self.dt)

        rospy.sleep(2)

	start = time.time()

        while True:
            (position, rotation) = self.get_odom()
            e_d = self.get_distance_error(position.x, position.y, self.x_d, self.y_d)
            e_theta = self.get_heading_error(position.x, position.y, rotation, self.x_d, self.y_d)

            self.pid_distance.setpoint = 0
            self.pid_heading.setpoint = 0

            linear_velocity = self.pid_distance(e_d)
            angular_velocity = self.pid_heading(e_theta)

            self.ys.append(position.y)
            self.xs.append(position.x)
	    self.ts.append(time.time()-start)

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

            rospy.sleep(self.dt)

            msg = Twist()
            msg.linear.x = 0
            msg.angular.z = 0
            self.cmd_vel.publish(msg)

        with open('{}.txt'.format(self.filename), 'w') as file:
            for x, y, t in zip(self.xs, self.ys, self.ts):
                file.write('{},{},{}\n'.format(x, y, t))

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

if __name__ == "__main__":
    data = [
        # kp, ki, kd, alpha, final_x, final_y, filename
        [1, 0.1, 0.05, 0.5, 0.5, 0.75, 'pid_nonlinear_fractional'],
    ]

    try:
        for kp, ki, kd, alpha, final_x, final_y, filename in data:
            initial_position = [0, 0, 0]
            q = quaternion_from_euler(0, 0, initial_position[2])
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

            ControlTurtlebot([0, 0], 0, [final_x, final_y], kp=kp, kd=kd, ki=ki, alpha=alpha, i=filename)
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("ControlTurtlebot node terminated.")




