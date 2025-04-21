import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

def calculate_area(xs, ys, x_d, y_d):
    m = y_d / x_d
    y_straight = [m * x for x in xs]
    difference = np.abs(np.array(ys) - np.array(y_straight))
    area = np.trapz(difference, xs)
    return area

def plot_trajectories(filenames, final_x, final_y, final_point_radius=0.06):
    plt.figure(figsize=(5, 5))
    metrics = {
        'Controller': [],
        'Error Area': [],
        'Sum of Errors': [],
        'Points Traversed': [],
        'Time Taken': []
    }

    for filename in filenames:
        xs, ys, ts = [], [], []
        with open(filename, 'r') as file:
            for line in file:
                x, y, t = map(float, line.strip().split(','))
                xs.append(x)
                ys.append(y)
                ts.append(t)

        plt.plot(xs, ys, label=filename[:-4])
        plt.scatter(xs[-1], ys[-1], marker='.', c='r', s=100)

        error_area = calculate_area(xs, ys, final_x, final_y)
        sum_of_errors = sum([np.sqrt((x-final_x)**2 + (y-final_y)**2) for x, y in zip(xs, ys)])
        points_traversed = len(xs)
        time_taken = ts[-1] if ts else 0

        metrics['Controller'].append(filename[:-4])
        metrics['Error Area'].append(error_area)
        metrics['Sum of Errors'].append(sum_of_errors)
        metrics['Points Traversed'].append(points_traversed)
        metrics['Time Taken'].append(time_taken)

    circle = plt.Circle((final_x, final_y), final_point_radius, color='r', alpha=0.2, label='Threshold Radius')
    plt.gca().add_patch(circle)

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim([0-0.3, final_x+0.3])
    plt.ylim([0-0.3, final_y+0.3])
    plt.legend()
    plt.grid(True)
    plt.title('Trajectories')
    plt.savefig('trajectories_plot.png')

    metrics_df = pd.DataFrame(metrics)
    metrics_df.to_csv('pid_metrics.csv', index=False)
    print(metrics_df)

    # Plot x vs. time
    plt.figure(figsize=(5, 5))
    for filename in filenames:
	thresholds = []
        xs, ys, ts = [], [], []
        with open(filename, 'r') as file:
            for line in file:
                x, y, t = map(float, line.strip().split(','))
		thresholds.append(final_x - 0.06)
                xs.append(x)
                ys.append(y)
                ts.append(t)

        plt.plot(ts, xs, label=filename[:-4])

    plt.plot(ts, thresholds, label='Threshold Line', color='red')
    plt.xlabel('Time')
    plt.ylabel('X')
    plt.legend()
    plt.grid(True)
    plt.title('X vs. Time')
    plt.savefig('x_vs_time_plot.png')

    # Plot y vs. time
    plt.figure(figsize=(5, 5))
    for filename in filenames:
	thresholds = []
        xs, ys, ts = [], [], []
        with open(filename, 'r') as file:
            for line in file:
                x, y, t = map(float, line.strip().split(','))
		thresholds.append(final_y - 0.06)
                xs.append(x)
                ys.append(y)
                ts.append(t)

        plt.plot(ts, ys, label=filename[:-4])

    plt.plot(ts, thresholds, label='Threshold Line', color='red')
    plt.xlabel('Time')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True)
    plt.title('Y vs. Time')
    plt.savefig('y_vs_time_plot.png')

if __name__ == "__main__":
    filenames = [
        'pid_normal.txt',
        'pid_fractional.txt',
        'pid_adaptive_fractional.txt',
        'pid_nonlinear_fractional.txt',
        'pid_time_delay_fractional.txt',
    ]
    final_x, final_y = 0.5, 0.5  # Final position for calculating straight line error
    plot_trajectories(filenames, final_x, final_y)

