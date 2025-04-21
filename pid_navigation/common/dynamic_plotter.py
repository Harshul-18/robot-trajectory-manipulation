#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import threading
import time
from matplotlib.animation import FuncAnimation

class DynamicTrajectoryPlotter:
    """
    Class for dynamically plotting robot trajectories in real-time
    """
    def __init__(self, title, expected_trajectory=None):
        """
        Initialize the dynamic plotter
        
        Args:
            title: Plot title
            expected_trajectory: List of (x, y) points for the expected trajectory (optional)
        """
        # Initialize data lists
        self.x_data = []
        self.y_data = []
        self.expected_x = []
        self.expected_y = []
        self.title = title
        
        # Set up the figure and axis
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title(title)
        self.ax.grid(True)
        
        # Create line objects
        self.actual_line, = self.ax.plot([], [], 'b-', label='Actual Trajectory')
        self.actual_point, = self.ax.plot([], [], 'bo', markersize=6)
        
        # If expected trajectory is provided, plot it
        if expected_trajectory is not None:
            self.expected_x = [point[0] for point in expected_trajectory]
            self.expected_y = [point[1] for point in expected_trajectory]
            self.ax.plot(self.expected_x, self.expected_y, 'r--', label='Expected Trajectory')
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Set axis limits with some padding
        if expected_trajectory is not None:
            x_min = min(self.expected_x) - 0.5
            x_max = max(self.expected_x) + 0.5
            y_min = min(self.expected_y) - 0.5
            y_max = max(self.expected_y) + 0.5
            self.ax.set_xlim(x_min, x_max)
            self.ax.set_ylim(y_min, y_max)
        else:
            self.ax.set_xlim(-2, 2)
            self.ax.set_ylim(-2, 2)
        
        # Make axes equal
        self.ax.set_aspect('equal')
        
        # Animation setup
        self.animation = None
        self.is_running = False
        self.thread = None
    
    def update_plot(self, frame):
        """
        Update function for the animation
        """
        if len(self.x_data) > 0:
            self.actual_line.set_data(self.x_data, self.y_data)
            self.actual_point.set_data(self.x_data[-1], self.y_data[-1])
        return self.actual_line, self.actual_point
    
    def add_point(self, x, y):
        """
        Add a new point to the trajectory
        
        Args:
            x: X coordinate
            y: Y coordinate
        """
        self.x_data.append(x)
        self.y_data.append(y)
        
        # Adjust axis limits if needed
        if x < self.ax.get_xlim()[0] or x > self.ax.get_xlim()[1]:
            self.ax.set_xlim(min(self.ax.get_xlim()[0], x - 0.5), max(self.ax.get_xlim()[1], x + 0.5))
        
        if y < self.ax.get_ylim()[0] or y > self.ax.get_ylim()[1]:
            self.ax.set_ylim(min(self.ax.get_ylim()[0], y - 0.5), max(self.ax.get_ylim()[1], y + 0.5))
    
    def start(self):
        """
        Start the animation in a separate thread
        """
        if not self.is_running:
            self.is_running = True
            self.thread = threading.Thread(target=self._run_animation)
            self.thread.daemon = True
            self.thread.start()
    
    def _run_animation(self):
        """
        Run the animation (called in a separate thread)
        """
        self.animation = FuncAnimation(
            self.fig, self.update_plot, interval=100, 
            blit=True, cache_frame_data=False
        )
        plt.show()
    
    def stop(self):
        """
        Stop the animation
        """
        self.is_running = False
        if self.animation is not None:
            self.animation.event_source.stop()
        plt.close(self.fig)
    
    def save_plot(self, filename):
        """
        Save the current plot to a file
        
        Args:
            filename: Output filename
        """
        plt.figure(figsize=(10, 8))
        plt.plot(self.x_data, self.y_data, 'b-', label='Actual Trajectory')
        
        if len(self.expected_x) > 0:
            plt.plot(self.expected_x, self.expected_y, 'r--', label='Expected Trajectory')
        
        if len(self.x_data) > 0:
            plt.plot(self.x_data[0], self.y_data[0], 'go', markersize=8, label='Start')
            plt.plot(self.x_data[-1], self.y_data[-1], 'ro', markersize=8, label='End')
        
        plt.grid(True)
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(self.title)
        plt.legend()
        plt.axis('equal')
        plt.savefig(filename)
        plt.close() 