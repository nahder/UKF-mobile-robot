"""
Nader Ahmed
Northwestern University, CS469

The motion model uses the translational and rotational velocity commands and performs 
dead-reckoning. Its performance is then drawn on a plot alongside the ground truth positions
of the robot. 

The measurement model returns the distance and bearing for a robot relative
to any landmark in the room (using their known positions) in the world frame. 

A UKF is used to correct for inaccuracies. This program will run all plots and testing.

"""

from motion_testing import test_sequence, compare_ground_truth
from measurement_model import test_measurement_model
from evaluation import test_UKF_ds1, test_UKF_ds0

def main(): 
    #plots motion model predictions for #2
    # test_sequence() 


    #computing range and bearings for a few landmarks for #6
    # test_measurement_model()

    #run UKF on sample commands
    # test_UKF_ds0()

    #ground truth vs motion model trajectories for #3
    compare_ground_truth() 

    #run UKF on robot data
    test_UKF_ds1()

if __name__ == "__main__": 
    main() 
