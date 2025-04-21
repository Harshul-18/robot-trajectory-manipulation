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

def generate_expected_line(start_point, end_point, num_points):
    """
    Generate expected trajectory points for a straight line
    
    Args:
        start_point: Starting point [x, y]
        end_point: Ending point [x, y]
        num_points: Number of points to generate
        
    Returns:
        List of expected trajectory points
    """
    x_start, y_start = start_point
    x_end, y_end = end_point
    
    # Generate points along the line
    t = np.linspace(0, 1, num_points)
    x = x_start + (x_end - x_start) * t
    y = y_start + (y_end - y_start) * t
    
    return list(zip(x, y))

def analyze_line_trajectory(trajectory_file, start_point, end_point, output_dir):
    """
    Analyze a line trajectory
    
    Args:
        trajectory_file: Path to the trajectory file
        start_point: Starting point [x, y]
        end_point: Ending point [x, y]
        output_dir: Output directory for analysis results
        
    Returns:
        Dictionary with analysis results
    """
    # Read trajectory data
    try:
        trajectory = read_trajectory_data(trajectory_file)
    except Exception as e:
        print("Error reading trajectory file: {}".format(trajectory_file))
        return None
    
    if trajectory is None:
        print("Error reading trajectory file: {}".format(trajectory_file))
        return None, None, None, None
    
    # Extract actual path
    actual_x = trajectory['x']
    actual_y = trajectory['y']
    actual_points = list(zip(actual_x, actual_y))
    
    # Generate expected path
    expected_points = generate_expected_line(start_point, end_point, len(actual_points))
    expected_x = [p[0] for p in expected_points]
    expected_y = [p[1] for p in expected_points]
    
    # Calculate errors
    rmse = calculate_rmse(actual_points, expected_points)
    mae = calculate_mae(actual_points, expected_points)
    time_taken = trajectory['time'][-1]
    
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
    plt.title("Line Trajectory - {}\nRMSE: {:.4f}, MAE: {:.4f}, Time: {:.2f}s, Score: {:.4f}".format(
        controller_type, rmse, mae, time_taken, score))
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    
    # Make axes equal
    plt.axis('equal')
    
    # Create directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Save plot
    output_file = os.path.join(output_dir, "line_{}_{}_{}.png".format(controller_type, end_point[0], end_point[1]))
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

def analyze_all_line_trajectories(data_dir, output_dir, controller_types, final_positions):
    """
    Analyze all line trajectories
    
    Args:
        data_dir: Directory containing trajectory data
        output_dir: Output directory for analysis results
        controller_types: List of controller types to analyze
        final_positions: List of final positions to analyze
        
    Returns:
        DataFrame with analysis results
    """
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize results list
    results = []
    
    # Analyze each trajectory
    for controller_type in controller_types:
        for final_position in final_positions:
            # Construct trajectory file path
            trajectory_file = os.path.join(data_dir, "line_{}_{}_{}.csv".format(controller_type, final_position[0], final_position[1]))
            
            if not os.path.exists(trajectory_file):
                print("Trajectory file not found: {}".format(trajectory_file))
                continue
            
            # Analyze trajectory
            result = analyze_line_trajectory(
                trajectory_file=trajectory_file,
                start_point=[0.0, 0.0],
                end_point=final_position,
                output_dir=output_dir
            )
            
            if result is not None:
                # Add final position to result
                result['final_x'] = final_position[0]
                result['final_y'] = final_position[1]
                
                # Add result to list
                results.append(result)
    
    # Create DataFrame from results
    results_df = pd.DataFrame(results)
    
    # Save results to CSV
    results_file = os.path.join(output_dir, "line_results.csv")
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
    summary_file = os.path.join(output_dir, "line_summary.csv")
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
    anova_file = os.path.join(output_dir, "line_anova.txt")
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
    tukey_file = os.path.join(output_dir, "line_tukey.txt")
    with open(tukey_file, 'w') as f:
        f.write("Tukey's HSD Test Results\n")
        f.write("=======================\n\n")
        
        for metric, result in tukey_results.items():
            f.write("{}:\n".format(metric))
            f.write(str(result.summary()) + "\n\n")

if __name__ == '__main__':
    # Define controller types
    controller_types = ['normal', 'fractional', 'adaptive', 'nonlinear', 'timedelay']
    
    # Define final positions
    final_positions = [
        [1.0, 0.0],
        [0.0, 1.0],
        [1.0, 1.0]
    ]
    
    # Define directories
    virtual_data_dir = os.path.join('..', 'results', 'virtual', 'line')
    virtual_output_dir = os.path.join('..', 'results', 'analysis', 'virtual', 'line')
    
    real_data_dir = os.path.join('..', 'results', 'real', 'line')
    real_output_dir = os.path.join('..', 'results', 'analysis', 'real', 'line')
    
    # Create output directories if they don't exist
    os.makedirs(virtual_output_dir, exist_ok=True)
    os.makedirs(real_output_dir, exist_ok=True)
    
    # Analyze virtual trajectories
    print("Analyzing virtual line trajectories...")
    virtual_results = analyze_all_line_trajectories(
        data_dir=virtual_data_dir,
        output_dir=virtual_output_dir,
        controller_types=controller_types,
        final_positions=final_positions
    )
    
    # Analyze real trajectories
    print("Analyzing real line trajectories...")
    real_results = analyze_all_line_trajectories(
        data_dir=real_data_dir,
        output_dir=real_output_dir,
        controller_types=controller_types,
        final_positions=final_positions
    )
    
    print("Line trajectory analysis completed.") 