#!/usr/bin/env python

import rospy
import time
import sys
import os
import numpy as np
import argparse
from geometry_msgs.msg import Twist

# Add the common directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'common'))

from robot_controller import RobotController
from pid_wrapper import PIDWrapper
from trajectory_utils import generate_line_waypoints, generate_square_waypoints, generate_ellipse_waypoints

class RealRobotController(RobotController):
    """
    Controller for the physical Turtlebot3 robot
    """
    def __init__(self, initial_position, theta, final_position, pid_controller, filename, dt=0.1, enable_dynamic_plot=True, expected_trajectory=None):
        """
        Initialize the real robot controller
        
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
        # Initialize base class
        super(RealRobotController, self).__init__(
            initial_position=initial_position,
            theta=theta,
            final_position=final_position,
            pid_controller=pid_controller,
            filename=filename,
            dt=dt,
            enable_dynamic_plot=enable_dynamic_plot,
            expected_trajectory=expected_trajectory
        )
        
        # Wait for robot to stabilize
        rospy.sleep(1.0)
        
        # Log initial position
        rospy.loginfo("Real robot initialized at position ({}, {}, {})".format(initial_position[0], initial_position[1], theta))

def run_line_trajectory(controller_type, final_position, num_points=16, output_dir=None, kp=0.5, ki=0.1, kd=0.2, alpha=0.5, enable_dynamic_plot=True):
    """
    Run a line trajectory experiment
    
    Args:
        controller_type: Type of PID controller
        final_position: Final position [x, y]
        num_points: Number of points for expected trajectory
        output_dir: Output directory
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        alpha: Fractional order
        enable_dynamic_plot: Whether to enable dynamic plotting
    """
    # Set initial position and orientation
    initial_position = [0.0, 0.0]
    theta = 0.0
    
    # Generate expected trajectory
    expected_trajectory = []
    if enable_dynamic_plot:
        expected_trajectory = [(initial_position[0] + t * (final_position[0] - initial_position[0]) / num_points,
                               initial_position[1] + t * (final_position[1] - initial_position[1]) / num_points)
                              for t in range(num_points + 1)]
    
    # Create output directory if it doesn't exist
    if output_dir is None:
        output_dir = os.path.join('..', 'results', 'real', 'line')
    os.makedirs(output_dir, exist_ok=True)
    
    # Create output filename
    filename = os.path.join(output_dir, "line_{}_{}_{}.csv".format(controller_type, final_position[0], final_position[1]))
    
    # Create PID controller
    pid_controller = PIDWrapper(
        controller_type=controller_type,
        kp=kp,
        ki=ki,
        kd=kd,
        alpha=alpha
    )
    
    # Create robot controller
    robot = RealRobotController(
        initial_position=initial_position,
        theta=theta,
        final_position=final_position,
        pid_controller=pid_controller,
        filename=filename,
        enable_dynamic_plot=enable_dynamic_plot,
        expected_trajectory=expected_trajectory
    )
    
    # Align heading
    rospy.loginfo("Aligning heading...")
    while not robot.align_heading() and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    # Move to target
    rospy.loginfo("Moving to target: {}...".format(final_position))
    start_time = time.time()
    while not robot.move_to_target(start_time) and not rospy.is_shutdown():
        rospy.sleep(0.1)
    
    # Save trajectory
    robot.save_trajectory()
    rospy.loginfo("Line trajectory completed in {:.2f} seconds".format(time.time() - start_time))

def run_square_trajectory(controller_type, side_length, num_points=16, output_dir=None, kp=0.5, ki=0.1, kd=0.2, alpha=0.5, enable_dynamic_plot=True):
    """
    Run a square trajectory experiment
    
    Args:
        controller_type: Type of PID controller
        side_length: Side length of the square
        num_points: Number of points for expected trajectory
        output_dir: Output directory
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        alpha: Fractional order
        enable_dynamic_plot: Whether to enable dynamic plotting
    """
    # Set initial position and orientation
    initial_position = [0.0, 0.0]
    theta = 0.0
    
    # Generate waypoints
    waypoints = [
        [0.0, 0.0],
        [side_length, 0.0],
        [side_length, side_length],
        [0.0, side_length],
        [0.0, 0.0]
    ]
    
    # Generate expected trajectory
    expected_trajectory = []
    if enable_dynamic_plot:
        # Generate points along each side of the square
        points_per_side = num_points // 4
        expected_trajectory = []
        
        for i in range(4):
            start = waypoints[i]
            end = waypoints[i+1]
            
            for j in range(points_per_side):
                t = j / points_per_side
                x = start[0] + t * (end[0] - start[0])
                y = start[1] + t * (end[1] - start[1])
                expected_trajectory.append((x, y))
        
        # Add the final point
        expected_trajectory.append((waypoints[4][0], waypoints[4][1]))
    
    # Create output directory if it doesn't exist
    if output_dir is None:
        output_dir = os.path.join('..', 'results', 'real', 'square')
    os.makedirs(output_dir, exist_ok=True)
    
    # Create output filename
    filename = os.path.join(output_dir, "square_{}_{}.csv".format(controller_type, side_length))
    
    # Create PID controller
    pid_controller = PIDWrapper(
        controller_type=controller_type,
        kp=kp,
        ki=ki,
        kd=kd,
        alpha=alpha
    )
    
    # Create robot controller
    robot = RealRobotController(
        initial_position=initial_position,
        theta=theta,
        final_position=initial_position,  # Start and end at the same point
        pid_controller=pid_controller,
        filename=filename,
        enable_dynamic_plot=enable_dynamic_plot,
        expected_trajectory=expected_trajectory
    )
    
    # Run trajectory
    start_time = time.time()
    
    for i in range(1, len(waypoints)):
        # Set target
        robot.final_position = waypoints[i]
        
        # Align heading
        rospy.loginfo("Aligning heading to waypoint {}...".format(i))
        while not robot.align_heading() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Move to target
        rospy.loginfo("Moving to waypoint {}: {}...".format(i, waypoints[i]))
        while not robot.move_to_target(start_time) and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    # Save trajectory
    robot.save_trajectory()
    rospy.loginfo("Square trajectory completed in {:.2f} seconds".format(time.time() - start_time))

def run_ellipse_trajectory(controller_type, a, b, num_points=16, output_dir=None, kp=0.5, ki=0.1, kd=0.2, alpha=0.5, enable_dynamic_plot=True):
    """
    Run an ellipse trajectory experiment
    
    Args:
        controller_type: Type of PID controller
        a: Semi-major axis length
        b: Semi-minor axis length
        num_points: Number of points for expected trajectory
        output_dir: Output directory
        kp: Proportional gain
        ki: Integral gain
        kd: Derivative gain
        alpha: Fractional order
        enable_dynamic_plot: Whether to enable dynamic plotting
    """
    # Set initial position and orientation
    initial_position = [a, 0.0]  # Start at the right side of the ellipse
    theta = 0.0
    
    # Generate waypoints
    waypoints = []
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points
        x = a * np.cos(angle)
        y = b * np.sin(angle)
        waypoints.append([x, y])
    
    # Add the first point again to close the loop
    waypoints.append(waypoints[0])
    
    # Generate expected trajectory
    expected_trajectory = []
    if enable_dynamic_plot:
        # Generate more points for smoother visualization
        for i in range(num_points * 4):
            angle = 2 * np.pi * i / (num_points * 4)
            x = a * np.cos(angle)
            y = b * np.sin(angle)
            expected_trajectory.append((x, y))
        
        # Add the first point again to close the loop
        expected_trajectory.append(expected_trajectory[0])
    
    # Create output directory if it doesn't exist
    if output_dir is None:
        output_dir = os.path.join('..', 'results', 'real', 'ellipse')
    os.makedirs(output_dir, exist_ok=True)
    
    # Create output filename
    filename = os.path.join(output_dir, "ellipse_{}_{}_{}.csv".format(controller_type, a, b))
    
    # Create PID controller
    pid_controller = PIDWrapper(
        controller_type=controller_type,
        kp=kp,
        ki=ki,
        kd=kd,
        alpha=alpha
    )
    
    # Create robot controller
    robot = RealRobotController(
        initial_position=initial_position,
        theta=theta,
        final_position=initial_position,  # Start and end at the same point
        pid_controller=pid_controller,
        filename=filename,
        enable_dynamic_plot=enable_dynamic_plot,
        expected_trajectory=expected_trajectory
    )
    
    # Run trajectory
    start_time = time.time()
    
    for i in range(1, len(waypoints)):
        # Set target
        robot.final_position = waypoints[i]
        
        # Align heading
        rospy.loginfo("Aligning heading to waypoint {}...".format(i))
        while not robot.align_heading() and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        # Move to target
        rospy.loginfo("Moving to waypoint {}: {}...".format(i, waypoints[i]))
        while not robot.move_to_target(start_time) and not rospy.is_shutdown():
            rospy.sleep(0.1)
    
    # Save trajectory
    robot.save_trajectory()
    rospy.loginfo("Ellipse trajectory completed in {:.2f} seconds".format(time.time() - start_time))

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run real robot experiments')
    parser.add_argument('--controller_type', type=str, default='normal', choices=['normal', 'fractional', 'adaptive', 'nonlinear', 'timedelay'], help='Type of PID controller')
    parser.add_argument('--trajectory', type=str, default='line', choices=['line', 'square', 'ellipse'], help='Type of trajectory')
    parser.add_argument('--final_position', type=str, default='1.0,0.0', help='Final position for line trajectory (x,y)')
    parser.add_argument('--side_length', type=float, default=1.0, help='Side length for square trajectory')
    parser.add_argument('--a', type=float, default=1.0, help='Semi-major axis length for ellipse trajectory')
    parser.add_argument('--b', type=float, default=0.5, help='Semi-minor axis length for ellipse trajectory')
    parser.add_argument('--num_points', type=int, default=16, help='Number of points for trajectory')
    parser.add_argument('--output_dir', type=str, default=None, help='Output directory')
    parser.add_argument('--kp', type=float, default=0.5, help='Proportional gain')
    parser.add_argument('--ki', type=float, default=0.1, help='Integral gain')
    parser.add_argument('--kd', type=float, default=0.2, help='Derivative gain')
    parser.add_argument('--alpha', type=float, default=0.5, help='Fractional order')
    parser.add_argument('--no_plot', action='store_true', help='Disable dynamic plotting')
    args = parser.parse_args()
    
    # Parse final position
    if args.trajectory == 'line':
        final_position = [float(x) for x in args.final_position.split(',')]
    
    # Run experiment
    try:
        if args.trajectory == 'line':
            run_line_trajectory(
                controller_type=args.controller_type,
                final_position=final_position,
                num_points=args.num_points,
                output_dir=args.output_dir,
                kp=args.kp,
                ki=args.ki,
                kd=args.kd,
                alpha=args.alpha,
                enable_dynamic_plot=not args.no_plot
            )
        elif args.trajectory == 'square':
            run_square_trajectory(
                controller_type=args.controller_type,
                side_length=args.side_length,
                num_points=args.num_points,
                output_dir=args.output_dir,
                kp=args.kp,
                ki=args.ki,
                kd=args.kd,
                alpha=args.alpha,
                enable_dynamic_plot=not args.no_plot
            )
        elif args.trajectory == 'ellipse':
            run_ellipse_trajectory(
                controller_type=args.controller_type,
                a=args.a,
                b=args.b,
                num_points=args.num_points,
                output_dir=args.output_dir,
                kp=args.kp,
                ki=args.ki,
                kd=args.kd,
                alpha=args.alpha,
                enable_dynamic_plot=not args.no_plot
            )
    except rospy.ROSInterruptException:
        pass 