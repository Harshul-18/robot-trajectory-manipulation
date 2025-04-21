mport pandas as pd
import numpy as np
import os
import cv2
import matplotlib.pyplot as plt

def world_to_pixel(x_world, y_world, map_origin, resolution, map_height):
    x_pixel = int((x_world - map_origin[0]) / resolution)
    y_pixel = map_height - int((y_world - map_origin[1]) / resolution)
    return x_pixel, y_pixel

def calculate_metrics(df):
    initial_position = df.iloc[0][['x', 'y']].values
    final_position = df.iloc[-1][['x', 'y']].values
   
    df['error'] = df.apply(lambda row: np.linalg.norm(row[['x', 'y']].values - initial_position), axis=1)
    df['sum_error'] = df.apply(lambda row: np.linalg.norm(row[['x', 'y']].values - final_position), axis=1)
    time_taken = df['t'].max()
   
    return df, time_taken

def load_and_analyze(filepath, map_image, map_origin, resolution):
    df = pd.read_csv(filepath, names=['x', 'y', 't'])
    df, time_taken = calculate_metrics(df)
   
    error = df['error'].mean()
    sum_of_errors = df['sum_error'].sum()

    print("Metrics:")
    print("Error: {}".format(error))
    print("Sum of Errors: {}".format(sum_of_errors))
    print("Time Taken: {}".format(time_taken))

    plot_trajectory(df, map_image, map_origin, resolution, filepath.replace('.txt', '_trajectory.png'))

    return df

def plot_trajectory(df, map_image, map_origin, resolution, save_path):
    height, width = map_image.shape
   
    # Convert world coordinates to pixel coordinates
    df['px'] = df.apply(lambda row: world_to_pixel(row['x'], row['y'], map_origin, resolution, height)[0], axis=1)
    df['py'] = df.apply(lambda row: world_to_pixel(row['x'], row['y'], map_origin, resolution, height)[1], axis=1)

    # Plot map background
    plt.figure(figsize=(10, 10))
    plt.imshow(map_image, cmap='gray')

    # Plot trajectory
    plt.plot(df['px'], df['py'], marker='o', linestyle='-', linewidth=2, color='blue', label='Path')
   
    # Mark the initial and final positions
    plt.plot(df['px'].iloc[0], df['py'].iloc[0], 'go', markersize=10, label='Start')  # Start position in green
    plt.plot(df['px'].iloc[-1], df['py'].iloc[-1], 'ro', markersize=10, label='End')  # End position in red

    plt.title('Trajectory on Map')
    plt.legend()
    plt.axis('off')
   
    # Save the plot
    plt.savefig(save_path, bbox_inches='tight')
    plt.close()

if __name__ == "__main__":
    filename = 'dynamic_obstacle_avoidance.txt'
   
    # Load the map image
    map_image = cv2.imread('map.pgm', cv2.IMREAD_GRAYSCALE)

    # Map configuration (adjust according to your map)
    resolution = 0.05  # meters per pixel
    map_origin = (-10, -10)  # origin of the map in world coordinates

    df = load_and_analyze(filename, map_image, map_origin, resolution)

    # Optionally save the DataFrame to a CSV file
    df.to_csv('dynamic_obstacle_avoidance_metrics.csv', index=False)
