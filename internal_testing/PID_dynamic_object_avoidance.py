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
    def __init__(self, initial_position, theta, final_position, kp, ki, kd, i, map_file, resolution, origin, dt=0.1):
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

        self.initial_position = initial_position
        self.final_position = final_position

        self.dt = dt

        self.map_data = self.load_map(map_file)
        self.resolution = resolution
        self.origin = origin

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

    def load_map(self, map_file):
        with open(map_file, 'r') as f:
            header = f.readline()
            assert header.strip() == 'P5', "Unknown file format"

            while True:
                line = f.readline()
                if line.startswith('#'):
                    continue
                else:
                    break

            width, height = map(int, line.split())
            max_val = int(f.readline())

            data = np.fromfile(f, dtype=np.uint8, count=width*height)
            map_data = np.reshape(data, (height, width))

        return map_data
    
    def world_to_pixel(self, position):
        x, y = position
        px = int((x - self.origin[0]) / self.resolution)
        py = 384 - int((y - self.origin[1]) / self.resolution)
        return (px, py)

    def pixel_to_world(self, pixel):
        px, py = pixel
        x = py * self.resolution + self.origin[0]
        y = (384-px) * self.resolution + self.origin[1]
        return (x, y)

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

            if abs(e_theta) < 0.02:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.cmd_vel.publish(msg)
                break

            msg = Twist()
            msg.angular.z = 0.5 * (-abs(e_theta)/e_theta)
            self.cmd_vel.publish(msg)


            rospy.sleep(dt)

            # msg = Twist()
            # msg.angular.z = 0
            # self.cmd_vel.publish(msg)

        # print(x_d, y_d)

        rospy.sleep(2)


        while True:
            (position, rotation) = self.get_odom()
            e_d = self.get_distance_error(position.x, position.y, x_d, y_d)
            e_theta = self.get_heading_error(position.x, position.y, rotation, x_d, y_d)

            # print(e_d, e_theta)

            pid_distance.setpoint = 0
            pid_heading.setpoint = 0

            linear_velocity = pid_distance(e_d)
            angular_velocity = pid_heading(e_theta)

            if self.msg is not None:

                if min(self.msg[-45:] + self.msg[:45]) < 0.3:
                    print("Turning")
                    linear_velocity = 0
                    angular_velocity = 0.5 * (-abs(e_theta)/e_theta)
                    msg = Twist()
                    msg.linear.x = linear_velocity
                    msg.angular.z = angular_velocity
                    self.cmd_vel.publish(msg)
                    while min(self.msg[-45:] + self.msg[:45]) < 0.3:
                        continue

                rospy.sleep(0.1)

                if (angular_velocity > 0):
                    if min(self.msg[-115:-45]) < 0.3:
                        print("Moving Forward - Watching Right")
                        linear_velocity = 0.1
                        angular_velocity = -0.1
                    msg = Twist()
                    msg.linear.x = linear_velocity
                    msg.angular.z = angular_velocity
                    self.cmd_vel.publish(msg)
                    while min(self.msg[-115:-45]) < 0.3:
                        continue
                elif (angular_velocity < 0):
                    if min(self.msg[45:115]) < 0.3:
                        print("Moving Forward - Watching Left")
                        linear_velocity = 0.1
                        angular_velocity = +0.1
                    msg = Twist()
                    msg.linear.x = linear_velocity
                    msg.angular.z = angular_velocity
                    self.cmd_vel.publish(msg)
                    while min(self.msg[45:115]) < 0.3:
                        continue

                rospy.sleep(0.1)

                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 0
                self.cmd_vel.publish(msg)

            # print("linear_velocity: {}; angular_velocity: {}; real_pos: {}; real_theta: {}".format(
            #     linear_velocity,
            #     angular_velocity,
            #     position,
            #     rotation
            # ))

            self.ys.append(position.y)
            self.xs.append(position.x)

            if self.is_shutdown:
                break

            if e_d < 0.1:
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

        plt.figure(figsize=(15, 5))

        height, width = self.map_data.shape
        world_coordinates = np.zeros((height, width, 2))

        for y_p in range(height):
            for x_p in range(width):
                x_w = x_p * resolution + origin[0]
                y_w = (384 - y_p) * resolution + origin[1]
                world_coordinates[y_p, x_p] = [x_w, y_w]

        x_coords = world_coordinates[:, :, 0]
        y_coords = world_coordinates[:, :, 1]


        fig, ax = plt.subplots(figsize=(8, 8))
        plt.scatter(x_coords, y_coords, s=1, c=self.map_data, cmap='gray')

        ax.plot(self.initial_position[0], self.initial_position[1], marker='o', color='black', markersize=5, label='Start')
        ax.plot(self.final_position[0], self.final_position[1], marker='o', color='green', markersize=5, label='Goal')

        plt.plot(self.xs, self.ys, label='Real Bot Trajectory')
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title('World Coordinates')
        plt.legend()
        plt.grid(True)
        plt.title(self.filename)

        if not os.path.exists('./plot_results'):
            os.mkdir('./plot_results')
        plt.savefig('./plot_results/{}.png'.format(self.filename))

        plt.show()

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
    [1, 0.1, 0.05, 0.5, -1.8, 'Dynamic Object Avoidance Trajectory'],
]

if __name__ == "__main__":
    try:
        i = 0
        for kp, ki, kd, final_x, final_y, filename in data:
            initial_position = [0.55, -2.05, 0]
            q = quaternion_from_euler(0, 0, initial_position[2])
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
            map_file = "map.pgm"
            resolution = 0.05 
            origin = [-10, -10] 

            error = ControlTurtlebot([initial_position[0], initial_position[1]], 0, [final_x, final_y], kp=kp, kd=kd, ki=ki, i=filename, map_file=map_file, resolution=resolution, origin=origin)
    except rospy.ROSInterruptException:
        rospy.loginfo("ControlTurtlebot node terminated.")
