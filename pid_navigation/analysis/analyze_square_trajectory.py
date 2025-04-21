#!/usr/bin/env python

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.stats import f_oneway
from statsmodels.stats.multicomp import pairwise_tukeyhsd

# Add the common directory to the Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'common'))

from trajectory_utils import read_trajectory_data, calculate_rmse, calculate_mae, calculate_score

def generate_expected_square(side_length, num_points):
    """
    Generate expected trajectory points for a square
    
    Args:
        side_length: Length of each side of the square
        num_points: Number of points to generate
        
    Returns:
        List of expected trajectory points
    """
    # Calculate number of points per side
    points_per_side = max(2, num_points // 4)
    
    # Generate points for each side
    sides = []
    
    # Bottom side: (0,0) to (side_length,0)
    t = np.linspace(0, 1, points_per_side)
    x = t * side_length
    y = np.zeros_like(t)
    sides.append(list(zip(x, y)))
    
    # Right side: (side_length,0) to (side_length,side_length)
    t = np.linspace(0, 1, points_per_side)
    x = np.ones_like(t) * side_length
    y = t * side_length
    sides.append(list(zip(x, y)))
    
    # Top side: (side_length,side_length) to (0,side_length)
    t = np.linspace(0, 1, points_per_side)
    x = side_length - t * side_length
    y = np.ones_like(t) * side_length
    sides.append(list(zip(x, y)))
    
    # Left side: (0,side_length) to (0,0)
    t = np.linspace(0, 1, points_per_side)
    x = np.zeros_like(t)
    y = side_length - t * side_length
    sides.append(list(zip(x, y)))
    
    # Combine all sides and remove duplicates
    points = []
    for side in sides:
        points.extend(side[:-1])  # Exclude the last point to avoid duplicates
    
    # Add the final point to close the loop
    points.append((0, 0))
    
    # Interpolate to get the desired number of points
    if len(points) != num_points:
        # Create a parameter along the path
        t_total = np.linspace(0, 1, len(points))
        t_desired = np.linspace(0, 1, num_points)
        
        # Interpolate x and y coordinates
        x = np.interp(t_desired, t_total, [p[0] for p in points])
        y = np.interp(t_desired, t_total, [p[1] for p in points])
        
        points = list(zip(x, y))
    
    return points

def analyze_square_trajectory(trajectory_file, side_length, output_dir):
    """
    Analyze a square trajectory
    
    Args:
        trajectory_file: Path to the trajectory file
        side_length: Length of each side of the square
        output_dir: Output directory for analysis results
        
    Returns:
        Dictionary with analysis results
    """
    # Read trajectory data
    try:
        trajectory_data = pd.read_csv(trajectory_file)
    except Exception as e:
        print("Error reading trajectory file: {}".format(trajectory_file))
        return None
    
    # Extract actual path
    actual_x = trajectory_data['x']
    actual_y = trajectory_data['y']
    actual_points = list(zip(actual_x, actual_y))
    
    # Generate expected path
    expected_points = generate_expected_square(side_length, len(actual_points))
    expected_x = [p[0] for p in expected_points]
    expected_y = [p[1] for p in expected_points]
    
    # Calculate errors
    rmse = calculate_rmse(actual_points, expected_points)
    mae = calculate_mae(actual_points, expected_points)
    time_taken = trajectory_data['time'][-1]
    
    # Calculate score
    score = calculate_score(rmse, mae, time_taken)
    
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
    
    # Add title with metrics
    controller_type = os.path.basename(trajectory_file).split('_')[1]
    plt.title("Square Trajectory - {}\nRMSE: {:.4f}, MAE: {:.4f}, Time: {:.2f}s, Score: {:.4f}".format(
        controller_type, rmse, mae, time_taken, score))
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    
    # Make axes equal
    plt.axis('equal')
    
    # Create directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Save plot
    output_file = os.path.join(output_dir, "square_{}_{}.png".format(controller_type, side_length))
    plt.savefig(output_file)
    plt.close()
    
    # Return analysis results
    return {
        'controller_type': controller_type,
        'rmse': rmse,
        'mae': mae,
        'time_taken': time_taken,
        'score': score
    }

def analyze_all_square_trajectories(data_dir, output_dir, controller_types, side_length):
    """
    Analyze all square trajectories
    
    Args:
        data_dir: Directory containing trajectory data
        output_dir: Output directory for analysis results
        controller_types: List of controller types to analyze
        side_length: Length of each side of the square
        
    Returns:
        DataFrame with analysis results
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize results list
    results = []
    
    # Analyze each trajectory
    for controller_type in controller_types:
        # Construct trajectory file path
        trajectory_file = os.path.join(data_dir, "square_{}_{}.csv".format(controller_type, side_length))
        
        if not os.path.exists(trajectory_file):
            print("Trajectory file not found: {}".format(trajectory_file))
            continue
        
        # Analyze trajectory
        result = analyze_square_trajectory(
            trajectory_file=trajectory_file,
            side_length=side_length,
            output_dir=output_dir
        )
        
        if result is not None:
            # Add side length to result
            result['side_length'] = side_length
            
            # Add result to list
            results.append(result)
    
    # Create DataFrame from results
    results_df = pd.DataFrame(results)
    
    # Save results to CSV
    results_file = os.path.join(output_dir, "square_results.csv")
    results_df.to_csv(results_file, index=False)
    
    # Perform statistical analysis
    perform_statistical_analysis(results_df, output_dir)
    
    return results_df

def perform_statistical_analysis(results_df, output_dir):
    """
    Perform statistical analysis on the results
    
    Args:
        results_df: DataFrame with analysis results
        output_dir: Output directory for analysis results
    """
    # Check if there are enough data points for statistical analysis
    if len(results_df) < 2:
        print("Not enough data points for statistical analysis")
        return
    
    # Group results by controller type
    grouped = results_df.groupby('controller_type')
    
    # Calculate mean and standard deviation for each metric
    summary = grouped.agg({
        'rmse': ['mean', 'std'],
        'mae': ['mean', 'std'],
        'time_taken': ['mean', 'std'],
        'score': ['mean', 'std']
    })
    
    # Save summary to CSV
    summary_file = os.path.join(output_dir, "square_summary.csv")
    summary.to_csv(summary_file)
    
    # Perform ANOVA test for each metric
    metrics = ['rmse', 'mae', 'time_taken', 'score']
    anova_results = {}
    
    for metric in metrics:
        # Extract data for each controller type
        data = [group[metric].values for name, group in grouped]
        
        # Perform ANOVA test
        f_stat, p_value = f_oneway(*data)
        
        # Store results
        anova_results[metric] = {
            'f_statistic': f_stat,
            'p_value': p_value
        }
    
    # Save ANOVA results to text file
    anova_file = os.path.join(output_dir, "square_anova.txt")
    with open(anova_file, 'w') as f:
        f.write("ANOVA Test Results\n")
        f.write("==================\n\n")
        
        for metric, result in anova_results.items():
            f.write("{}:\n".format(metric))
            f.write("  F-statistic: {:.4f}\n".format(result['f_statistic']))
            f.write("  p-value: {:.4f}\n".format(result['p_value']))
            f.write("  Significant: {}\n\n".format(result['p_value'] < 0.05))
    
    # Perform Tukey's HSD test for each metric
    tukey_results = {}
    
    for metric in metrics:
        # Create data for Tukey's test
        data = []
        groups = []
        
        for name, group in grouped:
            for value in group[metric].values:
                data.append(value)
                groups.append(name)
        
        # Perform Tukey's test
        tukey = pairwise_tukeyhsd(data, groups, alpha=0.05)
        
        # Store results
        tukey_results[metric] = tukey
    
    # Save Tukey's test results to text file
    tukey_file = os.path.join(output_dir, "square_tukey.txt")
    with open(tukey_file, 'w') as f:
        f.write("Tukey's HSD Test Results\n")
        f.write("=======================\n\n")
        
        for metric, result in tukey_results.items():
            f.write("{}:\n".format(metric))
            f.write(str(result.summary()) + "\n\n")

if __name__ == '__main__':
    # Define controller types
    controller_types = ['normal', 'fractional', 'adaptive', 'nonlinear', 'timedelay']
    
    # Define side length
    side_length = 1.0
    
    # Define directories
    virtual_data_dir = os.path.join('..', 'results', 'virtual', 'square')
    virtual_output_dir = os.path.join('..', 'results', 'analysis', 'virtual', 'square')
    
    real_data_dir = os.path.join('..', 'results', 'real', 'square')
    real_output_dir = os.path.join('..', 'results', 'analysis', 'real', 'square')
    
    # Create output directories if they don't exist
    os.makedirs(virtual_output_dir, exist_ok=True)
    os.makedirs(real_output_dir, exist_ok=True)
    
    # Analyze virtual trajectories
    print("Analyzing virtual square trajectories...")
    virtual_results = analyze_all_square_trajectories(
        data_dir=virtual_data_dir,
        output_dir=virtual_output_dir,
        controller_types=controller_types,
        side_length=side_length
    )
    
    # Analyze real trajectories
    print("Analyzing real square trajectories...")
    real_results = analyze_all_square_trajectories(
        data_dir=real_data_dir,
        output_dir=real_output_dir,
        controller_types=controller_types,
        side_length=side_length
    )
    
    print("Square trajectory analysis completed.") 