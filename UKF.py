import numpy as np
from motion_model import sampling_motion_model
from measurement_model import measurement_model, get_landmark_data
from scipy.linalg import sqrtm

class UKF:
    def __init__(self, initial_state, initial_covariance, process_noise, measurement_noise, alpha, beta, kappa):
        self.mu = initial_state
        self.sigma = initial_covariance

        self.Q = measurement_noise 
        self.R = process_noise
        self.R_measurement = np.diag([0.0001, 0.0001])  

        self.alpha = alpha
        self.beta = beta
        self.kappa = kappa
        self.n = len(initial_state)
        
        self.last_control_time = None
        self.landmark_data = get_landmark_data()

        self.lam = (self.alpha**2) * (self.n + self.kappa) - self.n

        self.weights_m = np.full(2*self.n + 1, 1 / (2 * (self.n + self.lam))) #mean weights, 2n+1 entries  shape, fill value
        self.weights_m[0] = self.lam / (self.n + self.lam) #m0

        self.weights_c = np.copy(self.weights_m) #c weights are = to mean weights except for 0th
        self.weights_c[0] = self.weights_c[0] + (1 - self.alpha**2 + self.beta)
        self.same_time=False
        self.delta_t = 0.12

    # Add a method to update delta_t when a new control command is received
    def update_delta_t(self, current_time):
        if self.last_control_time is not None:
            self.delta_t = current_time - self.last_control_time
        self.last_control_time = current_time


    def generate_sigma_points(self):
        #state: (x,y,theta). each sigma point: (x,y,theta)
        #given state vector of dim n, generate 2*n+1 sigma points 
        #create sigma points based on the current state and covariance

        root = sqrtm((self.n + self.lam) * self.sigma)
        root = root.real 

        sigma_points = np.zeros((2 * self.n + 1, self.n)) #sigma point matrix, each column = 1 sigma point 
        sigma_points[0] = self.mu #first column is current mean  
        for i in range(self.n):
            sigma_points[i + 1] = self.mu + root[i,:]#i=1 to n: after mean
            sigma_points[self.n + i + 1] = self.mu - root[i,:]#i=n+1 to 2n: before mean
        return sigma_points
    
    def motion_model(self, state, control):
        return sampling_motion_model(control, state, self.delta_t)
    
    def measurement_model(self, state, barcode_number):
        return measurement_model(state, barcode_number, self.landmark_data)

    def predict(self, control, current_time):
        if not self.same_time:
            self.update_delta_t(current_time)
        
        # transform sigma points via motion model -> chi bar
        transformed_points = []
        sigma_points = self.generate_sigma_points()
        for point in sigma_points:
            transformed_point = self.motion_model(point, control)
            transformed_points.append(transformed_point)

        # compute the weighted sum of transformed sigma points to get predicted mean 
        chi_bar = (np.array(transformed_points))

        predicted_mean = np.zeros_like(self.mu)
        for i in range(chi_bar.shape[0]):
            point = chi_bar[i,:]
            predicted_mean += self.weights_m[i] * point
        
        # compute predicted covariance 
        predicted_covariance = np.zeros_like(self.sigma) + self.R
        for i in range(chi_bar.shape[0]):
            deviation = chi_bar[i,:] - predicted_mean
            outer_product = np.outer(deviation, deviation)
            predicted_covariance += self.weights_c[i] * outer_product
        predicted_covariance += self.R

        # store the results for the next iteration
        self.mu = predicted_mean
        self.sigma = predicted_covariance

    def update(self, measurements):
        # generate sigma points around the updated mean and covariance
        sigma_points = self.generate_sigma_points()
        for measurement in measurements: #multiple measurements can be obsv same timestep

            barcode_number, *measurement_values = measurement
            measurement_values = np.array(measurement_values)

            # transform sigma points via measurement model
            transformed_measurements = []
            for point in sigma_points:
                transformed_measurement = self.measurement_model(point, barcode_number)
                transformed_measurements.append(transformed_measurement)
            zeta = np.array(transformed_measurements)

            # compute the weighted sum of transformed measurements
            zeta_bar = np.array([0.0,0.0])
            for i in range(zeta.shape[0]):
                zeta_bar += self.weights_m[i] * zeta[i,:]

            # compute the measurement covariance
            St = np.zeros((len(zeta_bar), len(zeta_bar))) + self.R_measurement
            for i in range(zeta.shape[0]):
                deviation = zeta[i,:] - zeta_bar
                outer_product = np.outer(deviation, deviation)
                St += self.weights_c[i] * outer_product

            St += self.Q
    
            # compute the cross-covariance between the state and the measurements
            Sigma_xz_t = np.zeros((len(self.mu), len(zeta_bar))) 
            for i in range(zeta.shape[0]):
                state_deviation = sigma_points[i] - self.mu
                measurement_deviation = zeta[i,:] - zeta_bar
                outer_product = np.outer(state_deviation, measurement_deviation)
                Sigma_xz_t += self.weights_c[i] * outer_product

            # compute Kalman Gain
            Kt = np.dot(Sigma_xz_t, np.linalg.inv(St))

            # update the state estimate
            self.mu += np.dot(Kt, (measurement_values - zeta_bar))

            # update the state covariance
            self.sigma -= np.dot(Kt, np.dot(St, Kt.T))
        # return the updated state and covariance
        return self.mu, self.sigma