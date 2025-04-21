#!/usr/bin/env python

import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def calculate_score(rmse, mae, time_taken):
    """
    Calculate a score for a controller based on RMSE, MAE, and time taken
    
    Args:
        rmse: Root Mean Square Error
        mae: Mean Absolute Error
        time_taken: Time taken to complete the trajectory
        
    Returns:
        Score (higher is better)
    """
    # Normalize errors (lower is better)
    normalized_rmse = 1.0 / (1.0 + rmse)
    normalized_mae = 1.0 / (1.0 + mae)
    
    # Normalize time (lower is better)
    normalized_time = 1.0 / (1.0 + time_taken)
    
    # Calculate score (higher is better)
    # Weight factors can be adjusted as needed
    rmse_weight = 0.4
    mae_weight = 0.4
    time_weight = 0.2
    
    score = (
        rmse_weight * normalized_rmse +
        mae_weight * normalized_mae +
        time_weight * normalized_time
    )
    
    return score

def calculate_line_scores(results_file):
    """
    Calculate scores for line trajectories
    
    Args:
        results_file: Path to the results file
        
    Returns:
        DataFrame with average scores grouped by controller
    """
    # Check if file exists
    if not os.path.exists(results_file):
        print("Results file not found: {}".format(results_file))
        return None
    
    # Read results
    results = pd.read_csv(results_file)
    
    # Group by controller type and calculate average scores
    grouped = results.groupby('controller_type')
    avg_scores = grouped.agg({
        'rmse': 'mean',
        'mae': 'mean',
        'time_taken': 'mean',
        'score': 'mean'
    })
    
    # Rename columns
    avg_scores = avg_scores.rename(columns={
        'rmse': 'line_rmse',
        'mae': 'line_mae',
        'time_taken': 'line_time',
        'score': 'line_score'
    })
    
    return avg_scores

def calculate_square_scores(results_file):
    """
    Calculate scores for square trajectories
    
    Args:
        results_file: Path to the results file
        
    Returns:
        DataFrame with average scores grouped by controller
    """
    # Check if file exists
    if not os.path.exists(results_file):
        print("Results file not found: {}".format(results_file))
        return None
    
    # Read results
    results = pd.read_csv(results_file)
    
    # Group by controller type and calculate average scores
    grouped = results.groupby('controller_type')
    avg_scores = grouped.agg({
        'rmse': 'mean',
        'mae': 'mean',
        'time_taken': 'mean',
        'score': 'mean'
    })
    
    # Rename columns
    avg_scores = avg_scores.rename(columns={
        'rmse': 'square_rmse',
        'mae': 'square_mae',
        'time_taken': 'square_time',
        'score': 'square_score'
    })
    
    return avg_scores

def calculate_ellipse_scores(results_file):
    """
    Calculate scores for ellipse trajectories
    
    Args:
        results_file: Path to the results file
        
    Returns:
        DataFrame with average scores grouped by controller
    """
    # Check if file exists
    if not os.path.exists(results_file):
        print("Results file not found: {}".format(results_file))
        return None
    
    # Read results
    results = pd.read_csv(results_file)
    
    # Group by controller type and calculate average scores
    grouped = results.groupby('controller_type')
    avg_scores = grouped.agg({
        'rmse': 'mean',
        'mae': 'mean',
        'time_taken': 'mean',
        'score': 'mean'
    })
    
    # Rename columns
    avg_scores = avg_scores.rename(columns={
        'rmse': 'ellipse_rmse',
        'mae': 'ellipse_mae',
        'time_taken': 'ellipse_time',
        'score': 'ellipse_score'
    })
    
    return avg_scores

def calculate_final_scores(line_scores, square_scores, ellipse_scores):
    """
    Calculate final scores for all controllers
    
    Args:
        line_scores: DataFrame with line scores
        square_scores: DataFrame with square scores
        ellipse_scores: DataFrame with ellipse scores
        
    Returns:
        DataFrame with final scores
    """
    # Merge scores
    final_scores = pd.DataFrame()
    
    if line_scores is not None:
        if final_scores.empty:
            final_scores = line_scores
        else:
            final_scores = final_scores.join(line_scores, how='outer')
    
    if square_scores is not None:
        if final_scores.empty:
            final_scores = square_scores
        else:
            final_scores = final_scores.join(square_scores, how='outer')
    
    if ellipse_scores is not None:
        if final_scores.empty:
            final_scores = ellipse_scores
        else:
            final_scores = final_scores.join(ellipse_scores, how='outer')
    
    # Fill NaN values with 0
    final_scores = final_scores.fillna(0)
    
    # Calculate final score
    # Weight factors can be adjusted as needed
    line_weight = 0.3
    square_weight = 0.35
    ellipse_weight = 0.35
    
    final_scores['final_score'] = (
        line_weight * final_scores['line_score'] +
        square_weight * final_scores['square_score'] +
        ellipse_weight * final_scores['ellipse_score']
    )
    
    # Sort by final score
    final_scores = final_scores.sort_values('final_score', ascending=False)
    
    return final_scores

def plot_scores(final_scores, output_file):
    """
    Plot scores for all controllers
    
    Args:
        final_scores: DataFrame with final scores
        output_file: Path to the output file
    """
    # Create figure
    plt.figure(figsize=(12, 10))
    
    # Get controller types
    controller_types = final_scores.index.tolist()
    
    # Create x positions
    x = np.arange(len(controller_types))
    width = 0.2
    
    # Plot line scores
    plt.bar(x - width, final_scores['line_score'], width, label='Line Score')
    
    # Plot square scores
    plt.bar(x, final_scores['square_score'], width, label='Square Score')
    
    # Plot ellipse scores
    plt.bar(x + width, final_scores['ellipse_score'], width, label='Ellipse Score')
    
    # Plot final scores
    plt.plot(x, final_scores['final_score'], 'ro-', linewidth=2, markersize=8, label='Final Score')
    
    # Add grid and legend
    plt.grid(True, axis='y')
    plt.legend()
    
    # Add title and labels
    plt.title('Controller Performance Comparison')
    plt.xlabel('Controller Type')
    plt.ylabel('Score (higher is better)')
    
    # Set x-axis ticks
    plt.xticks(x, controller_types)
    
    # Create directory if it doesn't exist
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    
    # Save figure
    plt.savefig(output_file)
    plt.close()

def main(environment):
    """
    Main function
    
    Args:
        environment: 'virtual' or 'real'
    """
    # Define directories
    results_dir = os.path.join('..', 'results', 'analysis', environment)
    output_dir = os.path.join('..', 'results', 'scores', environment)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Calculate scores for each trajectory type
    line_scores = calculate_line_scores(os.path.join(results_dir, 'line', 'line_results.csv'))
    square_scores = calculate_square_scores(os.path.join(results_dir, 'square', 'square_results.csv'))
    ellipse_scores = calculate_ellipse_scores(os.path.join(results_dir, 'ellipse', 'ellipse_results.csv'))
    
    # Calculate final scores
    final_scores = calculate_final_scores(line_scores, square_scores, ellipse_scores)
    
    # Save final scores to CSV
    final_scores.to_csv(os.path.join(output_dir, 'final_scores.csv'))
    
    # Plot scores
    plot_scores(final_scores, os.path.join(output_dir, 'scores_plot.png'))
    
    # Print final scores
    print("\nFinal Scores ({}):".format(environment))
    print(final_scores[['controller', 'line_score', 'square_score', 'ellipse_score', 'final_score']])
    
    # Print controller ranking
    print("\nController Ranking ({}):".format(environment))
    for i, (_, row) in enumerate(final_scores.iterrows()):
        print("{}. {}: {:.4f}".format(i+1, row['controller'], row['final_score']))

if __name__ == '__main__':
    # Check if environment is specified
    if len(sys.argv) > 1:
        environment = sys.argv[1]
        if environment not in ['virtual', 'real']:
            print("Invalid environment. Must be 'virtual' or 'real'.")
            sys.exit(1)
        
        # Run main function
        main(environment)
    else:
        # Run for both environments
        print("Running for virtual environment...")
        main('virtual')
        
        print("\nRunning for real environment...")
        main('real') 