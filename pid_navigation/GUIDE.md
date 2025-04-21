# Turtlebot-3 PID Navigation Project Guide

This guide provides step-by-step instructions for running all experiments and analysis tasks in the Turtlebot-3 PID navigation project.

## Prerequisites

Before starting, ensure you have the following:

1. ROS installed (Kinetic or newer)
2. Turtlebot3 packages installed
3. Gazebo simulator (for virtual experiments)
4. Python dependencies:
   - numpy
   - scipy
   - matplotlib
   - pandas
   - statsmodels

## Directory Structure

The project is organized as follows:

```
pid_navigation/
├── common/                 # Common utility modules
│   ├── pid_controllers.py  # PID controller implementations
│   ├── pid_wrapper.py      # PID controller wrapper
│   ├── robot_controller.py # Base robot controller
│   └── trajectory_utils.py # Trajectory utilities
├── virtual/                # Virtual robot implementation
│   └── virtual_robot.py    # Virtual robot controller
├── real/                   # Real robot implementation
│   └── real_robot.py       # Real robot controller
├── analysis/               # Analysis scripts
│   ├── analyze_line_trajectory.py    # Line trajectory analysis
│   ├── analyze_square_trajectory.py  # Square trajectory analysis
│   ├── analyze_ellipse_trajectory.py # Ellipse trajectory analysis
│   └── calculate_scores.py           # Score calculation
└── run_experiments.py      # Main script to run experiments
```

## Step 1: Make Scripts Executable

First, make all Python scripts executable:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
chmod +x run_experiments.py
chmod +x common/*.py
chmod +x virtual/*.py
chmod +x real/*.py
chmod +x analysis/*.py
```

## Step 2: Running Individual Experiments

### Virtual Environment Experiments

#### Line Trajectory

To run a line trajectory experiment with a specific controller in the virtual environment:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python virtual/virtual_robot.py --controller_type normal --final_position 1.0,0.0 --output_dir results/virtual/line
```

Available controller types:
- normal
- fractional
- adaptive
- nonlinear
- timedelay

You can change the final position by modifying the `--final_position` parameter.

#### Square Trajectory

To run a square trajectory experiment:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python virtual/virtual_robot.py --controller_type normal --side_length 1.0 --output_dir results/virtual/square
```

You can adjust the side length by modifying the `--side_length` parameter.

#### Ellipse Trajectory

To run an ellipse trajectory experiment:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python virtual/virtual_robot.py --controller_type normal --a 1.0 --b 0.5 --output_dir results/virtual/ellipse
```

Parameters:
- `--a`: Semi-major axis length
- `--b`: Semi-minor axis length

### Real Environment Experiments

#### Line Trajectory

To run a line trajectory experiment on the real Turtlebot3:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python real/real_robot.py --controller_type normal --final_position 1.0,0.0 --output_dir results/real/line
```

#### Square Trajectory

To run a square trajectory experiment on the real Turtlebot3:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python real/real_robot.py --controller_type normal --side_length 1.0 --output_dir results/real/square
```

#### Ellipse Trajectory

To run an ellipse trajectory experiment on the real Turtlebot3:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python real/real_robot.py --controller_type normal --a 1.0 --b 0.5 --output_dir results/real/ellipse
```

## Step 3: Running All Experiments

To run all experiments (both virtual and real) with all controller types:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python run_experiments.py
```

To run only virtual experiments:

```bash
python run_experiments.py --virtual
```

To run only real experiments:

```bash
python run_experiments.py --real
```

## Step 4: Running Analysis

### Analyzing Line Trajectory Results

To analyze line trajectory results:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python analysis/analyze_line_trajectory.py
```

This will:
1. Read trajectory data from the results directory
2. Calculate RMSE and MAE errors
3. Generate plots comparing actual and expected trajectories
4. Perform statistical analysis (ANOVA and Tukey's HSD tests)
5. Save results to CSV files and plots to image files

### Analyzing Square Trajectory Results

To analyze square trajectory results:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python analysis/analyze_square_trajectory.py
```

### Analyzing Ellipse Trajectory Results

To analyze ellipse trajectory results:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python analysis/analyze_ellipse_trajectory.py
```

### Calculating Final Scores

To calculate final scores and rank the controllers:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python analysis/calculate_scores.py
```

To calculate scores for a specific environment (virtual or real):

```bash
python analysis/calculate_scores.py virtual
# or
python analysis/calculate_scores.py real
```

## Step 5: Running All Analysis

To run all analysis scripts at once:

```bash
cd src/control_bot/Scripts/Finally_done/pid_navigation
python run_experiments.py --analysis
```

## Step 6: Viewing Results

After running the experiments and analysis, you can find the results in the following directories:

- Raw trajectory data: `results/virtual/` and `results/real/`
- Analysis results: `results/analysis/virtual/` and `results/analysis/real/`
- Final scores: `results/scores/virtual/` and `results/scores/real/`

The analysis results include:
- Plots comparing actual and expected trajectories
- CSV files with error metrics
- Statistical analysis results
- Final scores and rankings

## Troubleshooting

### Common Issues

1. **ROS node not initialized**: Make sure ROS is running with `roscore` before executing experiments.

2. **Gazebo not starting**: For virtual experiments, ensure Gazebo is properly installed and can be launched.

3. **Missing Python dependencies**: Install required packages with:
   ```bash
   pip install numpy scipy matplotlib pandas statsmodels
   ```

4. **Permission denied when running scripts**: Make sure all scripts are executable with `chmod +x`.

5. **Turtlebot3 not connecting**: For real experiments, ensure the Turtlebot3 is powered on and connected to the same network.

### Getting Help

If you encounter issues not covered in this guide, please refer to:
- The project README.md file
- ROS Wiki: http://wiki.ros.org/
- Turtlebot3 e-Manual: https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/

## Advanced Usage

### Customizing Parameters

You can customize experiment parameters by editing the `run_experiments.py` file:

```python
# Define parameters
controller_types = ['normal', 'fractional', 'adaptive', 'nonlinear', 'timedelay']
final_positions = [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0)]
side_length = 1.0
a = 1.0  # Semi-major axis
b = 0.5  # Semi-minor axis
num_points = 16
```

### Adding New Controllers

To add a new controller:

1. Implement the controller class in `common/pid_controllers.py`
2. Add the controller to the `PIDWrapper` class in `common/pid_wrapper.py`
3. Add the controller type to the `controller_types` list in `run_experiments.py`

### Adding New Trajectories

To add a new trajectory type:

1. Implement the trajectory generation function in `common/trajectory_utils.py`
2. Create trajectory execution functions in `virtual/virtual_robot.py` and `real/real_robot.py`
3. Create an analysis script in the `analysis` directory
4. Update the `run_experiments.py` script to include the new trajectory type 