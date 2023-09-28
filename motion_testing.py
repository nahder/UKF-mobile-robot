from motion_model import * 
import numpy as np

def read_dat_file(file_path):
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('#'):
                continue

            columns = line.strip().split()
            data_row = [float(col) for col in columns]
            data.append(data_row)

    return np.array(data)  


def test_sequence():
    x_prev = (0.0, 0.0, 0.0)
    x_positions = []
    y_positions = []

    commands = [
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 1.0),
        (0.0, -1/(2*np.pi), 2.0),
        (0.5, 0.0, 3.0),
        (0.0, 1/(2*np.pi), 4.0),
        (0.5, 0.0, 5.0)
    ]

    for i in range(len(commands)):
        command = commands[i]

        if i == 0:
            prev_timestamp = 0.0
        else:
            prev_timestamp = commands[i - 1][2]

        current_timestamp = command[2]

        delta_t = current_timestamp - prev_timestamp  

        x, y, theta = sampling_motion_model(command, x_prev, delta_t)
        x_positions.append(x)
        y_positions.append(y)
        x_prev = (x, y, theta)

    # Plot the trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(x_positions, y_positions, marker='o', linestyle='-')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Robot Trajectory')
    plt.grid(True)
    plt.show()

def main(): 
    # test_sequence()

    # Define file paths
    command_file = 'datasets/ds0/ds0_Odometry.dat'
    ground_truth_file = 'datasets/ds0/ds0_Groundtruth.dat'

    commands = read_dat_file(command_file)  # Array columns: time, fwd vel, angular vel
    ground_truth_data = read_dat_file(ground_truth_file)  # Array columns: time, x, y, heading

    ground_truth_x = ground_truth_data[0:, 1] #all rows col 1
    ground_truth_y = ground_truth_data[0:, 2]
    ground_truth_thetas = ground_truth_data[0:,3]

    x_prev = (ground_truth_x[0], ground_truth_y[0], ground_truth_thetas[0])
    x_positions = []
    y_positions = []

    for i in range(1, len(commands)):
        delta_t = commands[i][0] - commands[i-1][0] 
       
        v, w = commands[i][1], commands[i][2]
        x, y, theta = sampling_motion_model((v, w), x_prev, delta_t)

        x_positions.append(x)
  
        y_positions.append(y)

        x_prev = (x, y, theta)  

    plt.plot(ground_truth_x, ground_truth_y,label='Ground Truth Trajectory') 
    plt.plot(x_positions, y_positions,label='Motion Model Trajectory')

    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Trajectory Comparison: Motion Model vs Ground Truth')
    plt.legend()
    plt.show()



main() 