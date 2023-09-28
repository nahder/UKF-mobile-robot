""""
Design a motion model to estimate the positional (2-D location and heading) changes of a wheeled mobile
robot in response to commanded speed (translational and rotational) controls. Clearly define the inputs, outputs
and parameters of this model, and report the math
"""

import numpy as np 
import matplotlib.pyplot as plt 
 
def sampling_motion_model(control_command, x_prev, delta_t): 

    a = [.0000001, .0000001, .0000001, .000001, .000001, .0000001] #alphas

    v, w = control_command[0], control_command[1]

    if v ==0 and w==0: 
        return x_prev 
    
    x, y, theta = x_prev #start with 0 0 0, estimate 1st, and propagate forward 
    v_hat = v + sample_normal_distribution(a[0]*v**2 + a[1]*w**2) 
    w_hat = w + sample_normal_distribution(a[2]*v**2 + a[3]*w**2) 
    gamma_hat = sample_normal_distribution(a[4]*v**2 + a[5]*w**2)

    x_new = x - (v_hat/w_hat)*np.sin(theta) + (v_hat/w_hat)*np.sin(theta + (w_hat * delta_t) )
    y_new = y + (v_hat/w_hat)*np.cos(theta) - (v_hat/w_hat)*np.cos(theta + (w_hat * delta_t) )
    
    theta_new = theta + w_hat*delta_t + gamma_hat*delta_t 


    return x_new, y_new, theta_new 


def sample_normal_distribution(b):
    # generate an array of 12 random values sampled from a uniform distribution
    random_values = np.random.uniform(-b, b, 12)
    
    return 0.5 * np.sum(random_values)



# def motion_model_simple(control_command, x_prev, delta_t): 

#     x_0, y_0, v_i, w_i, t_i = x_prev 
#     v_f, w_f, t_f = control_command #setting desired velocities and timestamp 

#     delta_d = .5*(v_i + v_f) * delta_t 

#     delta_theta = .5*(w_i + w_f) * delta_t 

#     robot_x = x_0 + delta_d*np.cos(delta_theta) 
#     robot_y = y_0 + delta_d*np.sin(delta_theta) 

#     return robot_x,robot_y,delta_theta 



