""""
Design a motion model to estimate the positional (2-D location and heading) changes of a wheeled mobile
robot in response to commanded speed (translational and rotational) controls. Clearly define the inputs, outputs
and parameters of this model, and report the math
"""

import numpy as np 
import matplotlib.pyplot as plt 

# read in odometry data (times, fwd velocities (m/s), angular velocities (m/s))

# motion model: state transition model in mobile robotics
# p(x_t | u_t, x_{t-1})

# velocity motion model: control a robot through translational (v_t) and angular velocities (w_t). u_t = [v_t, w_t]
# w_t: ccw + , cw - w 


# p = motion_model_vel(x_new, u_t, x) 
# return the probability of arriving at x_t given x_prev and control input 


def motion_model_vel(x_t, u_t, x_prev, delta_t): 

    #inputs
    v_t, w_t = u_t 
    x, y,theta = x_prev 
    x_p, y_p, theta_p = x_t 
    
    #circle parameters
    mu = .5 * ((x-x_p)*np.cos(theta) + (y-y_p)*np.sin(theta)) / ((y-y_p)*np.cos(theta) - (x-x_p)*np.sin(theta)) 
    x_star = .5 * (x+x_p) + mu(y-y_p) 
    y_star = .5 * (y+y_p) + mu(x_p-x) 
    r_star = np.sqrt( (x-x_star)**2 + (y-y_star)**2 ) 
    delta_theta = np.atan2(y_p-y_star, x_p-x_star) - np.atan2(y-y_star, x-x_star) 
     
    #actual velocities 
    v_hat = delta_theta / delta_t 
    w_hat = delta_theta / delta_t 
    gamma_hat = ((theta_p - theta) / delta_t) - w_hat 


    

def prob_normal_distribution(a,b): 

    return (1 / np.sqrt(2*np.pi*b)) * np.exp(-.5 * a**2 / b ) 
