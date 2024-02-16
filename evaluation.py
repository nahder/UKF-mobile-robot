import numpy as np 

from UKF import UKF 
import matplotlib.pyplot as plt

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

def load_ground_truth_data(filename):
    data_array = read_dat_file(filename)
    data = {}
    for row in data_array:
        timestamp = row[0]
        state = (row[1], row[2], row[3])  
        data[timestamp] = state
    return data

def load_control_data(filename):
    data_array = read_dat_file(filename)
    data = {}
    for row in data_array:
        timestamp = row[0]
        control = (row[1], row[2])  
        data[timestamp] = control
    return data

def load_measurement_data(filename):
    data_array = read_dat_file(filename)
    data = {}

    ignored_barcodes = {5, 14, 41, 32, 23}  # Set for faster lookup
    
    for row in data_array:
        timestamp = row[0]
        barcode = int(row[1])
        
        # Check if the barcode is not in the ignored list
        if barcode not in ignored_barcodes:
            measurement = (barcode, row[2], row[3])  
            if timestamp in data:
                data[timestamp].append(measurement)
            else:
                data[timestamp] = [measurement]    
    return data


def test_UKF_ds1():
    command_file = 'datasets/ds1/ds1_Odometry.dat'
    ground_truth_positions_file = 'datasets/ds1/ds1_Groundtruth.dat'
    measurements_file = 'datasets/ds1/ds1_Measurement.dat'

    # Load data
    control_data = load_control_data(command_file)  # time: (linear_vel, angular_vel)
    ground_truth_data = load_ground_truth_data(ground_truth_positions_file)  # time: (x, y, theta)
    measurement_data = load_measurement_data(measurements_file)

    initial_state = np.array([0.98038490, -4.99232180, 1.44849633])  # first ground truth state
    initial_covariance = np.array([
        [0.05, 0.0001, 0.0001],
        [0.0001, 0.05, 0.0001],
        [0.0001, 0.0001, 0.05]
    ])
    measurement_noise = np.diag([.15,3.0])  # R sensor
    process_noise = np.diag([.001, .001, .068])  # Q
    alpha = 0.00001
    beta = 2
    kappa = 0.0

    filter = UKF(initial_state=initial_state, initial_covariance=initial_covariance,
                process_noise=process_noise, measurement_noise=measurement_noise,
                alpha=alpha, beta=beta, kappa=kappa)

    estimated_trajectory = []

    # create combined list of timestamps
    combined_timestamps = sorted(set(list(control_data.keys()) + list(measurement_data.keys())))

    for t in combined_timestamps:
        if t in control_data:
            filter.same_time = False
            control = control_data[t]
            filter.predict(control, t)
        else:
            # if there's no control data at this timestamp, set control and delta_t to 0
            filter.predict([0, 0], t)
            filter.same_time=True 

        estimated_trajectory.append([t] + list(filter.mu))


        if t in measurement_data:
            measurements_at_t = measurement_data[t]
            filter.update(measurements_at_t)


    estimated_trajectory = np.array(estimated_trajectory)


    ground_truth_trajectory = np.array(list(ground_truth_data.values()))

    plt.figure()
    plt.plot(estimated_trajectory[:, 1], estimated_trajectory[:, 2], label='Estimated Trajectory')
    plt.plot(ground_truth_trajectory[:, 0], ground_truth_trajectory[:, 1], label='Ground Truth Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Ground Truth vs. Estimated Trajectory via UKF')
    plt.legend()
    plt.show()

def test_UKF_ds0():
    commands = [ #(v, w, t)
        (0.0, 0.0, 0.0),
        (0.5, 0.0, 1.0),
        (0.0, -1/(2*np.pi), 2.0),
        (0.5, 0.0, 3.0),
        (0.0, 1/(2*np.pi), 4.0),
        (0.5, 0.0, 5.0)
    ]
    measurement_noise = np.diag([0.0,0.0])  # Q sensor

    initial_state = np.array([0.0, 0.0, 0.0])  # first ground truth state
    initial_covariance = np.array([
        [0.05, 0.0001, 0.0001],
        [0.0001, 0.05, 0.0001],
        [0.0001, 0.0001, 0.05]
    ])
    process_noise = np.diag([.001, .001, .068])  # R 
    alpha = 0.1
    beta = 2
    kappa = 0.0


    filter = UKF(initial_state=initial_state, initial_covariance=initial_covariance,
                process_noise=process_noise, alpha=alpha, beta=beta, kappa=kappa,measurement_noise=measurement_noise)

    estimated_trajectory = []

    for v, w, t in commands:
        filter.predict([v, w], t)
        estimated_trajectory.append([t] + list(filter.mu))

    estimated_trajectory = np.array(estimated_trajectory)

    plt.figure()
    plt.gca().set_aspect('equal')

    plt.plot(estimated_trajectory[:, 1], estimated_trajectory[:, 2], label='Estimated Trajectory')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Estimated Trajectory for Sample Commands (UKF Applied)')
    plt.show()


