import matplotlib.pyplot as plt
import numpy as np

def read_coordinates(filename):
    xs, ys, ts = [], [], []
    with open(filename, 'r') as file:
        for line in file:
            x, y, t = map(float, line.strip().split(','))
            xs.append(x)
            ys.append(y)
            ts.append(t)
    return np.array(xs), np.array(ys), np.array(ts)

def generate_expected_coordinates(length, a, b):
    thetas = np.linspace(0, 2*np.pi, length)
    x_expected = a * np.cos(thetas)
    y_expected = b * np.sin(thetas)
    return x_expected, y_expected

def plot_coordinates(xs, ys, ts, x_expected, y_expected):
    plt.figure(figsize=(15, 4))
   
    plt.subplot(1, 3, 1)
    plt.plot(xs, ys, '-o', markersize=2, label='Obtained Path')
    plt.plot(x_expected, y_expected, '-o', markersize=2, label='Theoretical Path')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.title('x vs y')
   
    plt.subplot(1, 3, 2)
    plt.plot(ts, xs, '-o', markersize=2, label='Obtained x')
    plt.plot(ts, x_expected, '-o', markersize=2, label='Theoretical x')
    plt.xlabel('t')
    plt.ylabel('x')
    plt.legend()
    plt.grid(True)
    plt.title('x vs t')
   
    plt.subplot(1, 3, 3)
    plt.plot(ts, ys, '-o', markersize=2, label='Obtained y')
    plt.plot(ts, y_expected, '-o', markersize=2, label='Theoretical y')
    plt.xlabel('t')
    plt.ylabel('y')
    plt.legend()
    plt.grid(True)
    plt.title('y vs t')
   
    plt.tight_layout()
    plt.savefig('ellipse_virtual.png')
    plt.show()

def compute_rmse(xs, ys, x_expected, y_expected):
    rmse_x = np.sqrt(np.mean((xs - x_expected)**2))
    rmse_y = np.sqrt(np.mean((ys - y_expected)**2))
    return rmse_x, rmse_y

if __name__ == "__main__":
    filename = 'pid_fractional_ellipse_virtual.txt'
    a = 1.0  # Semi-major axis
    b = 0.5  # Semi-minor axis

    xs, ys, ts = read_coordinates(filename)
    x_expected, y_expected = generate_expected_coordinates(len(xs), a, b)
    plot_coordinates(xs, ys, ts, x_expected, y_expected)
   
    rmse_x, rmse_y = compute_rmse(xs, ys, x_expected, y_expected)
    print('RMSE in x:', rmse_x)
    print('RMSE in y:', rmse_y)
    print('Correlation in x:', np.corrcoef(xs, x_expected)[1,0])
    print('Correlation in y:', np.corrcoef(ys, y_expected)[1,0])
    print('Time Taken:', ts[-1])
