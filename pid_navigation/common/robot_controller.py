#!/usr/bin/env python

import time
import math
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Point
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import tf
import os

# Import the dynamic plotter
from dynamic_plotter import DynamicTrajectoryPlotter

class RobotController:
    """
    Base class for controlling the Turtlebot3 robot
    """
    def __init__(self, initial_position, theta, final_position, pid_controller, filename, dt=0.1, enable_dynamic_plot=True, expected_trajectory=None):
        """
        Initialize the robot controller
        
        Args:
            initial_position: Initial position [x, y]
            theta: Initial orientation
            final_position: Final position [x, y]
            pid_controller: PID controller
            filename: Output filename
            dt: Time step
            enable_dynamic_plot: Whether to enable dynamic plotting
            expected_trajectory: Expected trajectory points for plotting
        """
        # Initialize ROS node
        rospy.init_node('turtlebot3_controller', anonymous=True)
        
        # Set up publisher and subscriber
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('scan', LaserScan, self.callback)
        
        # Initialize TF listener
        self.tf_listener = tf.TransformListener()
        
        # Initialize variables
        self.initial_position = initial_position
        self.theta = theta
        self.final_position = final_position
        self.pid_controller = pid_controller
        self.filename = filename
        self.dt = dt
        
        # Initialize trajectory data
        self.trajectory = {
            'time': [],
            'x': [],
            'y': [],
            'theta': [],
            'v': [],
            'w': []
        }
        
        # Initialize obstacle detection
        self.obstacle_detected = False
        self.min_distance = float('inf')
        
        # Initialize dynamic plotter if enabled
        self.enable_dynamic_plot = enable_dynamic_plot
        self.plotter = None
        
        if self.enable_dynamic_plot:
            # Extract controller type from filename
            controller_type = os.path.basename(filename).split('_')[1] if '_' in os.path.basename(filename) else 'unknown'
            plot_title = "Turtlebot3 Trajectory - {}".format(controller_type)
            
            # Create and start the plotter
            self.plotter = DynamicTrajectoryPlotter(plot_title, expected_trajectory)
            self.plotter.start()
        
        # Set up shutdown hook
        rospy.on_shutdown(self.shutdown)
    
    def get_odom(self):
        """
        Get odometry data from TF
        
        Returns:
            Tuple of (x, y, theta)
        """
        try:
            (trans, rot) = self.tf_listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
            x = trans[0]
            y = trans[1]
            
            (roll, pitch, theta) = euler_from_quaternion(rot)
            
            return (x, y, theta)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("TF Exception")
            return None
    
    def get_distance_error(self, x, y, x_d, y_d):
        """
        Calculate distance error
        
        Args:
            x: Current x position
            y: Current y position
            x_d: Desired x position
            y_d: Desired y position
            
        Returns:
            Distance error
        """
        return math.sqrt((x_d - x) ** 2 + (y_d - y) ** 2)
    
    def get_heading_error(self, x, y, theta, x_d, y_d):
        """
        Calculate heading error
        
        Args:
            x: Current x position
            y: Current y position
            theta: Current orientation
            x_d: Desired x position
            y_d: Desired y position
            
        Returns:
            Heading error
        """
        # Calculate desired heading
        theta_d = math.atan2(y_d - y, x_d - x)
        
        # Calculate heading error
        error = theta_d - theta
        
        # Normalize to [-pi, pi]
        while error > math.pi:
            error -= 2 * math.pi
        while error < -math.pi:
            error += 2 * math.pi
        
        return error
    
    def align_heading(self):
        """
        Align the robot heading with the target
        
        Returns:
            True if aligned, False otherwise
        """
        # Get current position and orientation
        odom_data = self.get_odom()
        if odom_data is None:
            return False
        
        x, y, theta = odom_data
        
        # Calculate heading error
        heading_error = self.get_heading_error(x, y, theta, self.final_position[0], self.final_position[1])
        
        # Check if heading is aligned
        if abs(heading_error) < 0.05:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            return True
        
        # Update heading setpoint
        self.pid_controller.update_heading_setpoint(0)
        
        # Compute control signal
        angular_z = self.pid_controller.compute_heading_control(heading_error)
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        return False
    
    def move_to_target(self, start_time=None):
        """
        Move the robot to the target position
        
        Args:
            start_time: Start time (if None, current time is used)
            
        Returns:
            True if target reached, False otherwise
        """
        # Get current position and orientation
        odom_data = self.get_odom()
        if odom_data is None:
            return False
        
        x, y, theta = odom_data
        
        # Calculate distance and heading errors
        distance_error = self.get_distance_error(x, y, self.final_position[0], self.final_position[1])
        heading_error = self.get_heading_error(x, y, theta, self.final_position[0], self.final_position[1])
        
        # Check if target is reached
        if distance_error < 0.05:
            # Stop the robot
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            self.cmd_vel_pub.publish(twist)
            return True
        
        # Update setpoints
        self.pid_controller.update_distance_setpoint(0)
        self.pid_controller.update_heading_setpoint(0)
        
        # Compute control signals
        linear_x = self.pid_controller.compute_distance_control(distance_error)
        angular_z = self.pid_controller.compute_heading_control(heading_error)
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)
        
        # Record trajectory data
        if start_time is None:
            start_time = time.time()
        
        current_time = time.time() - start_time
        self.trajectory['time'].append(current_time)
        self.trajectory['x'].append(x)
        self.trajectory['y'].append(y)
        self.trajectory['theta'].append(theta)
        self.trajectory['v'].append(linear_x)
        self.trajectory['w'].append(angular_z)
        
        # Update dynamic plot if enabled
        if self.enable_dynamic_plot and self.plotter is not None:
            self.plotter.add_point(x, y)
        
        return False
    
    def save_trajectory(self, mode='w'):
        """
        Save trajectory data to a file
        
        Args:
            mode: File open mode ('w' for write, 'a' for append)
        """
        # Create directory if it doesn't exist
        os.makedirs(os.path.dirname(self.filename), exist_ok=True)
        
        # Save trajectory data to CSV file
        with open(self.filename, mode) as f:
            # Write header
            if mode == 'w':
                f.write('time,x,y,theta,v,w\n')
            
            # Write data
            for i in range(len(self.trajectory['time'])):
                f.write("{},{},{},{},{},{}\n".format(self.trajectory['time'][i], self.trajectory['x'][i], self.trajectory['y'][i], self.trajectory['theta'][i], self.trajectory['v'][i], self.trajectory['w'][i]))
        
        # Save plot if dynamic plotting is enabled
        if self.enable_dynamic_plot and self.plotter is not None:
            plot_filename = self.filename.replace('.csv', '.png')
            self.plotter.save_plot(plot_filename)
    
    def callback(self, msg):
        """
        Callback function for laser scan
        
        Args:
            msg: LaserScan message
        """
        # Check for obstacles
        ranges = np.array(msg.ranges)
        ranges = ranges[~np.isnan(ranges) & ~np.isinf(ranges)]
        
        if len(ranges) > 0:
            self.min_distance = np.min(ranges)
            self.obstacle_detected = self.min_distance < 0.3
        else:
            self.obstacle_detected = False
            self.min_distance = float('inf')
    
    def shutdown(self):
        """
        Shutdown function
        """
        # Stop the robot
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        self.cmd_vel_pub.publish(twist)
        
        # Save trajectory data
        self.save_trajectory()
        
        # Stop the plotter
        if self.enable_dynamic_plot and self.plotter is not None:
            self.plotter.stop()
        
        rospy.loginfo("Robot stopped") 