#!/usr/bin/env python

import math
import numpy as np
import os
import matplotlib.pyplot as plt
from scipy.stats import f_oneway
from statsmodels.stats.multicomp import pairwise_tukeyhsd

def generate_line_waypoints(start_point, end_point):
    """
    Generate waypoints for a straight line trajectory
    
    Args:
        start_point: Starting point [x, y]
        end_point: Ending point [x, y]
        
    Returns:
        List of waypoints
    """
    return [start_point, end_point]

def generate_square_waypoints(origin, side_length):
    """
    Generate waypoints for a square trajectory
    
    Args:
        origin: Origin point [x, y]
        side_length: Length of each side
        
    Returns:
        List of waypoints
    """
    x0, y0 = origin
    
    # Define the four corners of the square
    waypoints = [
        [x0, y0],  # Starting point
        [x0 + side_length, y0],  # Bottom right
        [x0 + side_length, y0 + side_length],  # Top right
        [x0, y0 + side_length],  # Top left
        [x0, y0]  # Back to starting point
    ]
    
    return waypoints

def generate_ellipse_waypoints(origin, a, b, num_points=16):
    """
    Generate waypoints for an elliptical trajectory
    
    Args:
        origin: Origin point [x, y]
        a: Semi-major axis length
        b: Semi-minor axis length
        num_points: Number of waypoints to generate
        
    Returns:
        List of waypoints
    """
    x0, y0 = origin
    
    # Generate points along the ellipse
    waypoints = []
    for i in range(num_points + 1):  # +1 to close the loop
        angle = 2 * math.pi * i / num_points
        x = x0 + a * math.cos(angle)
        y = y0 + b * math.sin(angle)
        waypoints.append([x, y])
    
    return waypoints

def read_trajectory_data(filename):
    """
    Read trajectory data from a CSV file
    
    Args:
        filename: Path to the CSV file
        
    Returns:
        Dictionary with trajectory data
    """
    try:
        data = np.genfromtxt(filename, delimiter=',', names=True)
        
        trajectory = {
            'time': data['time'],
            'x': data['x'],
            'y': data['y'],
            'theta': data['theta'],
            'linear_velocity': data['linear_velocity'],
            'angular_velocity': data['angular_velocity']
        }
        
        return trajectory
    except Exception as e:
        print("Error reading trajectory data: {}".format(e))
        return None

def calculate_rmse(actual, expected):
    """
    Calculate the Root Mean Square Error (RMSE)
    
    Args:
        actual: Actual values
        expected: Expected values
        
    Returns:
        RMSE value
    """
    if len(actual) != len(expected):
        raise ValueError("Actual and expected arrays must have the same length")
    
    squared_errors = [(a - e)**2 for a, e in zip(actual, expected)]
    mean_squared_error = sum(squared_errors) / len(squared_errors)
    
    return math.sqrt(mean_squared_error)

def calculate_mae(actual, expected):
    """
    Calculate the Mean Absolute Error (MAE)
    
    Args:
        actual: Actual values
        expected: Expected values
        
    Returns:
        MAE value
    """
    if len(actual) != len(expected):
        raise ValueError("Actual and expected arrays must have the same length")
    
    absolute_errors = [abs(a - e) for a, e in zip(actual, expected)]
    
    return sum(absolute_errors) / len(absolute_errors)

def calculate_path_errors(trajectory_file, expected_path_func):
    """
    Calculate path errors for a trajectory
    
    Args:
        trajectory_file: Path to the trajectory file
        expected_path_func: Function to generate expected path points
        
    Returns:
        Dictionary with RMSE, MAE, and time taken
    """
    # Read trajectory data
    trajectory = read_trajectory_data(trajectory_file)
    if trajectory is None:
        return None
    
    # Extract actual path
    actual_x = trajectory['x']
    actual_y = trajectory['y']
    
    # Generate expected path
    expected_points = expected_path_func(len(actual_x))
    expected_x = [p[0] for p in expected_points]
    expected_y = [p[1] for p in expected_points]
    
    # Calculate errors
    rmse_x = calculate_rmse(actual_x, expected_x)
    rmse_y = calculate_rmse(actual_y, expected_y)
    rmse_total = math.sqrt(rmse_x**2 + rmse_y**2)
    
    mae_x = calculate_mae(actual_x, expected_x)
    mae_y = calculate_mae(actual_y, expected_y)
    mae_total = (mae_x + mae_y) / 2
    
    # Calculate time taken
    time_taken = trajectory['time'][-1]
    
    return {
        'rmse': rmse_total,
        'mae': mae_total,
        'time_taken': time_taken
    }

def plot_trajectory(trajectory_file, expected_path_func, title, output_file):
    """
    Plot the actual and expected trajectories
    
    Args:
        trajectory_file: Path to the trajectory file
        expected_path_func: Function to generate expected path points
        title: Plot title
        output_file: Path to save the plot
        
    Returns:
        True if successful, False otherwise
    """
    # Read trajectory data
    trajectory = read_trajectory_data(trajectory_file)
    if trajectory is None:
        return False
    
    # Extract actual path
    actual_x = trajectory['x']
    actual_y = trajectory['y']
    
    # Generate expected path
    expected_points = expected_path_func(len(actual_x))
    expected_x = [p[0] for p in expected_points]
    expected_y = [p[1] for p in expected_points]
    
    # Calculate errors
    rmse = calculate_rmse(list(zip(actual_x, actual_y)), expected_points)
    mae = calculate_mae(list(zip(actual_x, actual_y)), expected_points)
    time_taken = trajectory['time'][-1]
    
    # Create figure
    plt.figure(figsize=(10, 8))
    
    # Plot actual trajectory
    plt.plot(actual_x, actual_y, 'b-', label='Actual')
    
    # Plot expected trajectory
    plt.plot(expected_x, expected_y, 'r--', label='Expected')
    
    # Add start and end points
    plt.plot(actual_x[0], actual_y[0], 'go', label='Start')
    plt.plot(actual_x[-1], actual_y[-1], 'ro', label='End')
    
    # Add grid and legend
    plt.grid(True)
    plt.legend()
    
    # Add title and labels
    plt.title("{}\nRMSE: {:.4f}, MAE: {:.4f}, Time: {:.2f}s".format(title, rmse, mae, time_taken))
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    
    # Make axes equal
    plt.axis('equal')
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # Save figure
    plt.savefig(output_file)
    plt.close()
    
    return True

def calculate_score(rmse, mae, time_taken):
    """
    Calculate a score based on RMSE, MAE, and time taken
    
    Args:
        rmse: Root Mean Square Error
        mae: Mean Absolute Error
        time_taken: Time taken to complete the trajectory
        
    Returns:
        Score value (higher is better)
    """
    # Avoid division by zero
    denominator = rmse + mae + time_taken
    if denominator == 0:
        return 0
    
    return 1.0 / denominator

def analyze_results(results_files, output_file):
    """
    Analyze results from multiple controllers
    
    Args:
        results_files: Dictionary mapping controller names to result files
        output_file: Path to save the analysis results
        
    Returns:
        True if successful, False otherwise
    """
    # Read results
    results = {}
    for controller, file_path in results_files.items():
        try:
            data = np.genfromtxt(file_path, delimiter=',', names=True)
            results[controller] = {
                'rmse': data['rmse'],
                'mae': data['mae'],
                'time_taken': data['time_taken'],
                'score': data['score']
            }
        except Exception as e:
            print("Error reading results for {}: {}".format(controller, e))
            return False
    
    # Calculate average scores
    avg_scores = {}
    for controller, data in results.items():
        avg_scores[controller] = np.mean(data['score'])
    
    # Sort controllers by average score
    sorted_controllers = sorted(avg_scores.items(), key=lambda x: x[1], reverse=True)
    
    # Create figure
    plt.figure(figsize=(12, 8))
    
    # Plot average scores
    controllers = [c[0] for c in sorted_controllers]
    scores = [c[1] for c in sorted_controllers]
    
    plt.bar(controllers, scores)
    
    # Add title and labels
    plt.title('Average Scores by Controller')
    plt.xlabel('Controller')
    plt.ylabel('Average Score')
    
    # Add grid
    plt.grid(True, axis='y')
    
    # Rotate x-axis labels
    plt.xticks(rotation=45)
    
    # Adjust layout
    plt.tight_layout()
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # Save figure
    plt.savefig(output_file)
    plt.close()
    
    return True 