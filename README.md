# Robot Trajectory Manipulation

A comprehensive ROS-based project for trajectory planning, control, and navigation of TurtleBot3 robots in both simulated and real environments. This repository contains implementations of various PID controller types and machine learning approaches for robot trajectory manipulation and obstacle avoidance.

![Turtlebot Trajectory Control](Turtlebot%20Trajectory%20Control.png)

## Overview

This project includes a suite of tools for controlling TurtleBot3 robots along different trajectory types (linear, square, elliptical) using various control algorithms:

- **Standard PID Controllers**
- **Fractional PID Controllers**
- **Adaptive PID Controllers**
- **Nonlinear PID Controllers**
- **Time-Delay PID Controllers**

The project supports both simulated environments (in Gazebo) and real-world TurtleBot3 robots.

## Repository Structure

```
├── PID_Virtual/             # Virtual environment PID controllers
│   ├── pid_normal.py
│   ├── pid_fractional.py
│   ├── pid_adaptive_fractional.py
│   ├── pid_nonlinear_fractional.py
│   └── pid_time_delay_fractional.py
├── PID_Real/                # Real robot PID controllers
├── dynamic_virtual/         # Dynamic trajectory generation (virtual)
├── dynamic_real/            # Dynamic trajectory generation (real)
├── square/                  # Square trajectory implementations
├── ellipse/                 # Elliptical trajectory implementations
├── pid_navigation/          # Navigation modules
│   ├── common/              # Common utilities
│   ├── virtual/             # Virtual robot implementations
│   ├── real/                # Real robot implementations
│   └── analysis/            # Analysis scripts
├── ML/                      # Machine Learning implementations
│   ├── botrl.py             # Reinforcement learning for robot control
│   └── rnode.py             # ROS node for ML integration
└── static/                  # Static Obstacle avoidance
```

## Requirements

### Software Requirements

- ROS (Kinetic or noetic)
- Python (Python 2 for kinetic and Python 3 for noetic)
- Gazebo (for simulation)
- TurtleBot3 packages

### Python Dependencies

```
numpy
scipy
matplotlib
pandas
statsmodels
torch
gym
```

## Installation

1. Clone this repository into your catkin workspace's `src` directory:

```bash
cd ~/catkin_ws/src
git clone https://github.com/Harshul-18/robot-trajectory-manipulation.git
```

2. Build your catkin workspace:

```bash
cd ~/catkin_ws
catkin_make
```

3. Source your workspace:

```bash
source ~/catkin_ws/devel/setup.bash
```

4. Install Python dependencies:

```bash
pip install numpy scipy matplotlib pandas statsmodels torch gym
```

5. Make sure all scripts are executable:

```bash
cd ~/catkin_ws/src/robot-trajectory-manipulation
chmod +x PID_Virtual/*.py PID_Real/*.py pid_navigation/**/*.py ML/*.py
```

## Usage

### Starting the Simulation Environment

First, launch the Gazebo simulation environment with TurtleBot3:

```bash
# Launch TurtleBot3 in an empty world
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch

# Or launch TurtleBot3 in a pre-defined world with obstacles
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

### Running PID Controllers in Simulation

To run a basic PID controller in the virtual environment:

```bash
# Using rosrun for the standard PID controller
rosrun control_bot pid_normal.py

# For other controller types
rosrun control_bot pid_fractional.py
rosrun control_bot pid_adaptive_fractional.py
rosrun control_bot pid_nonlinear_fractional.py
rosrun control_bot pid_time_delay_fractional.py
```

### Running Different Trajectory Types

#### Linear Trajectory

```bash
# Launch the virtual robot with a linear trajectory using normal PID
rosrun control_bot virtual_robot.py --controller_type normal --final_position 1.0,0.0 --output_dir results/virtual/line
```

#### Square Trajectory

```bash
# Launch the virtual robot with a square trajectory using normal PID
rosrun control_bot virtual_robot.py --controller_type normal --side_length 1.0 --output_dir results/virtual/square
```

#### Elliptical Trajectory

```bash
# Launch the virtual robot with an elliptical trajectory using normal PID
rosrun control_bot virtual_robot.py --controller_type normal --a 1.0 --b 0.5 --output_dir results/virtual/ellipse
```

### Running with Real TurtleBot3

First, set up your TurtleBot3:

```bash
# On the TurtleBot3 (Remote PC)
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

Then, on your workstation:

```bash
# For basic PID control on real hardware
rosrun control_bot real_robot.py --controller_type normal --final_position 1.0,0.0
```

### Visualization in RViz

To visualize the robot's trajectory and sensor data:

```bash
roslaunch control_bot display_map.launch
```

### Reinforcement Learning

To train a reinforcement learning model for trajectory control:

```bash
rosrun control_bot botrl.py
```

### Running Analysis Tools

The project includes comprehensive analysis tools for evaluating controller performance:

```bash
# Analyze linear trajectory performance
rosrun control_bot analyze_line_trajectory.py

# Analyze square trajectory performance
rosrun control_bot analyze_square_trajectory.py

# Analyze elliptical trajectory performance
rosrun control_bot analyze_ellipse_trajectory.py

# Calculate performance scores across controllers
rosrun control_bot calculate_scores.py
```

## Features

- Multiple PID controller implementations (Classic, Fractional, Adaptive, etc.)
- Support for different trajectory types (Linear, Square, Elliptical)
- Virtual (simulated) and real robot implementations
- Comprehensive analysis tools for performance evaluation

## License

MIT License

## Author

[Harshul-18](https://github.com/Harshul-18)
