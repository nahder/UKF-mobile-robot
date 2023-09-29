"""
Nader Ahmed
Northwestern University, CS469
Assignment 1, Part A: Below is an implementation for the (1) motion and (2) measurement 
models for a robot roaming in a room. 

The motion model uses the translational and rotational velocity commands and performs 
dead-reckoning. Its performance is then drawn on a plot alongside the ground truth positions
of the robot. 

The measurement model returns the distance and bearing for a robot relative
to any landmark in the room (using their known positions) in the world frame. 

"""

from motion_testing import test_sequence, compare_ground_truth
from measurement_model import test_measurement_model

def main(): 
    test_sequence() 

    compare_ground_truth()  

    test_measurement_model()

if __name__ == "__main__": 
    main() 
