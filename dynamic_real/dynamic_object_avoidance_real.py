#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import math
import time
import numpy as np
from geometry_msgs.msg import Twist, Point
import tf
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
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from scipy.special import gamma
import os

class FractionalPID:
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

        output = self.Kp * error + self.Ki * integral + self.Kd * derivative

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

class DynamicObstacleAvoidance:
    def __init__(self, initial_position, final_position, kp, ki, kd, alpha, filename, dt=0.1):
        rospy.init_node('dynamic_obstacle_avoidance', anonymous=True)
        rospy.loginfo('To stop the turtlebot, press ctrl+c')
       
        self.is_shutdown = False
        self.cmd_vel = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

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
       
        self.xs = []
        self.ys = []
        self.ts = []

        self.x_d, self.y_d = final_position
        self.x, self.y = position.x, position.y
        self.theta = rotation

        self.dt = dt
        self.filename = filename

        self.pid_distance = FractionalPID(Kp=kp, Ki=ki, Kd=kd, alpha=alpha, setpoint=0)
        self.pid_heading = FractionalPID(Kp=kp, Ki=ki, Kd=kd, alpha=alpha, setpoint=0)
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

    def laser_callback(self, msg):
        self.msg = msg.ranges
   
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
	
	prev = (-abs(e_theta/e_theta))

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

	    if self.msg is not None:
                while min(self.msg[-45:] + self.msg[:45]) < 0.5:
                    angular_velocity = 0.5 * prev
                    linear_velocity = 0
		    msg = Twist()
		    msg.linear.x = linear_velocity
		    msg.angular.z = angular_velocity
		    self.cmd_vel.publish(msg)
		    if not (min(self.msg[-25:] + self.msg[:25]) < 0.5):
			prev = (-abs(e_theta/e_theta))

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

        with open('{}.txt'.format(self.filename), 'w') as f:
            for x, y, t in zip(self.xs, self.ys, self.ts):
                f.write('{},{},{}\n'.format(x, y, t))

        rospy.on_shutdown(self.shutdown)

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
        kp = 1
        ki = 0.1
        kd = 0.05
        alpha = 0.5
        final_x = 1
        final_y = 0.25
        filename = 'dynamic_obstacle_avoidance'

        initial_position = [-1.5, 0.5, 0]

        DynamicObstacleAvoidance(initial_position, [final_x, final_y], kp, ki, kd, alpha, filename)
    except rospy.ROSInterruptException:
        rospy.loginfo("DynamicObstacleAvoidance node terminated.")
