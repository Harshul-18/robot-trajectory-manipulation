#!/usr/bin/env python

import os
import sys
import argparse
import rospy
import time

# Add virtual and real directories to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), 'virtual'))
sys.path.append(os.path.join(os.path.dirname(__file__), 'real'))

# Import trajectory functions
from line_trajectory import run_line_trajectory
from square_trajectory import run_square_trajectory
from ellipse_trajectory import run_ellipse_trajectory

def run_virtual_experiments(controller_types, final_positions, side_length, a, b, num_points, output_dir, enable_dynamic_plot=True):
    """
    Run virtual experiments
    
    Args:
        controller_types: List of controller types to test
        final_positions: List of final positions for line trajectories
        side_length: Side length for square trajectories
        a: Semi-major axis for ellipse trajectories
        b: Semi-minor axis for ellipse trajectories
        num_points: Number of points for trajectories
        output_dir: Output directory for results
        enable_dynamic_plot: Whether to enable dynamic plotting
    """
    # Create output directories
    line_dir = os.path.join(output_dir, 'line')
    square_dir = os.path.join(output_dir, 'square')
    ellipse_dir = os.path.join(output_dir, 'ellipse')
    
    os.makedirs(line_dir, exist_ok=True)
    os.makedirs(square_dir, exist_ok=True)
    os.makedirs(ellipse_dir, exist_ok=True)
    
    # Run line trajectory experiments
    print("\nRunning virtual line trajectory experiments...")
    for controller_type in controller_types:
        for final_position in final_positions:
            print("  Running {} controller to position {}...".format(controller_type, final_position))
            
            # Run line trajectory
            run_line_trajectory(
                controller_type=controller_type,
                final_position=final_position,
                num_points=num_points,
                output_dir=line_dir,
                enable_dynamic_plot=enable_dynamic_plot
            )
            
            # Allow robot to stabilize
            rospy.sleep(2.0)
    
    # Run square trajectory experiments
    print("\nRunning virtual square trajectory experiments...")
    for controller_type in controller_types:
        print("  Running {} controller for square trajectory...".format(controller_type))
        
        # Run square trajectory
        run_square_trajectory(
            controller_type=controller_type,
            side_length=side_length,
            num_points=num_points,
            output_dir=square_dir,
            enable_dynamic_plot=enable_dynamic_plot
        )
        
        # Allow robot to stabilize
        rospy.sleep(2.0)
    
    # Run ellipse trajectory experiments
    print("\nRunning virtual ellipse trajectory experiments...")
    for controller_type in controller_types:
        print("  Running {} controller for ellipse trajectory...".format(controller_type))
        
        # Run ellipse trajectory
        run_ellipse_trajectory(
            controller_type=controller_type,
            a=a,
            b=b,
            num_points=num_points,
            output_dir=ellipse_dir,
            enable_dynamic_plot=enable_dynamic_plot
        )
        
        # Allow robot to stabilize
        rospy.sleep(2.0)

def run_real_experiments(controller_types, final_positions, side_length, a, b, num_points, output_dir, enable_dynamic_plot=True):
    """
    Run real experiments
    
    Args:
        controller_types: List of controller types to test
        final_positions: List of final positions for line trajectories
        side_length: Side length for square trajectories
        a: Semi-major axis for ellipse trajectories
        b: Semi-minor axis for ellipse trajectories
        num_points: Number of points for trajectories
        output_dir: Output directory for results
        enable_dynamic_plot: Whether to enable dynamic plotting
    """
    # Create output directories
    line_dir = os.path.join(output_dir, 'line')
    square_dir = os.path.join(output_dir, 'square')
    ellipse_dir = os.path.join(output_dir, 'ellipse')
    
    os.makedirs(line_dir, exist_ok=True)
    os.makedirs(square_dir, exist_ok=True)
    os.makedirs(ellipse_dir, exist_ok=True)
    
    # Run line trajectory experiments
    print("\nRunning real line trajectory experiments...")
    for controller_type in controller_types:
        for final_position in final_positions:
            print("  Running {} controller to position {}...".format(controller_type, final_position))
            
            # Run line trajectory
            run_line_trajectory(
                controller_type=controller_type,
                final_position=final_position,
                num_points=num_points,
                output_dir=line_dir,
                enable_dynamic_plot=enable_dynamic_plot
            )
            
            # Allow robot to stabilize
            rospy.sleep(2.0)
    
    # Run square trajectory experiments
    print("\nRunning real square trajectory experiments...")
    for controller_type in controller_types:
        print("  Running {} controller for square trajectory...".format(controller_type))
        
        # Run square trajectory
        run_square_trajectory(
            controller_type=controller_type,
            side_length=side_length,
            num_points=num_points,
            output_dir=square_dir,
            enable_dynamic_plot=enable_dynamic_plot
        )
        
        # Allow robot to stabilize
        rospy.sleep(2.0)
    
    # Run ellipse trajectory experiments
    print("\nRunning real ellipse trajectory experiments...")
    for controller_type in controller_types:
        print("  Running {} controller for ellipse trajectory...".format(controller_type))
        
        # Run ellipse trajectory
        run_ellipse_trajectory(
            controller_type=controller_type,
            a=a,
            b=b,
            num_points=num_points,
            output_dir=ellipse_dir,
            enable_dynamic_plot=enable_dynamic_plot
        )
        
        # Allow robot to stabilize
        rospy.sleep(2.0)

def run_analysis():
    """
    Run analysis scripts
    """
    # Run line trajectory analysis
    print("\nRunning line trajectory analysis...")
    os.system("python analysis/analyze_line_trajectory.py")
    
    # Run square trajectory analysis
    print("\nRunning square trajectory analysis...")
    os.system("python analysis/analyze_square_trajectory.py")
    
    # Run ellipse trajectory analysis
    print("\nRunning ellipse trajectory analysis...")
    os.system("python analysis/analyze_ellipse_trajectory.py")
    
    # Calculate scores
    print("\nCalculating scores...")
    os.system("python analysis/calculate_scores.py")

def main():
    """
    Main function
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run PID navigation experiments')
    parser.add_argument('--virtual', action='store_true', help='Run virtual experiments')
    parser.add_argument('--real', action='store_true', help='Run real experiments')
    parser.add_argument('--analysis', action='store_true', help='Run analysis scripts')
    parser.add_argument('--no_plot', action='store_true', help='Disable dynamic plotting')
    args = parser.parse_args()
    
    # Define parameters
    controller_types = ['normal', 'fractional', 'adaptive', 'nonlinear', 'timedelay']
    final_positions = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
    side_length = 1.0
    a = 1.0  # Semi-major axis
    b = 0.5  # Semi-minor axis
    num_points = 16
    
    # Define output directories
    virtual_output_dir = os.path.join('results', 'virtual')
    real_output_dir = os.path.join('results', 'real')
    
    # Initialize ROS node
    rospy.init_node('pid_navigation_experiments')
    
    # Run experiments
    if args.virtual or (not args.virtual and not args.real and not args.analysis):
        run_virtual_experiments(
            controller_types=controller_types,
            final_positions=final_positions,
            side_length=side_length,
            a=a,
            b=b,
            num_points=num_points,
            output_dir=virtual_output_dir,
            enable_dynamic_plot=not args.no_plot
        )
    
    if args.real or (not args.virtual and not args.real and not args.analysis):
        run_real_experiments(
            controller_types=controller_types,
            final_positions=final_positions,
            side_length=side_length,
            a=a,
            b=b,
            num_points=num_points,
            output_dir=real_output_dir,
            enable_dynamic_plot=not args.no_plot
        )
    
    if args.analysis or (not args.virtual and not args.real and not args.analysis):
        run_analysis()

if __name__ == '__main__':
    main() 