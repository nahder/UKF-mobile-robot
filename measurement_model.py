import numpy as np 
from motion_testing import read_dat_file 


def get_landmark_data(): 

    landmark_ground_truth = read_dat_file('datasets/ds1/ds1_Landmark_Groundtruth.dat')

    landmark_dict = {row[0]: (row[1], row[2]) for row in landmark_ground_truth}

    return landmark_dict 

def convert_barcode_to_subject(barcode): 
    barcodes_file = 'datasets/ds1/ds1_Barcodes.dat'

    barcodes_data = read_dat_file(barcodes_file)

    subjects = barcodes_data[:,0]
    barcodes = barcodes_data[:,1]
    barcode_dict = {}

    for i in range(len(barcodes)): 
        barcode_dict[barcodes[i]]=subjects[i]

    return barcode_dict[barcode]

def distance(p1, p2): 

    return np.linalg.norm(np.array(p1)-np.array(p2))

def measurement_model(cur_pose, barcode_number, landmark_data): 

    x, y, theta = cur_pose
    subject_number = convert_barcode_to_subject(barcode_number)
    landmark_x, landmark_y = landmark_data[subject_number]

    # calculate range
    r = distance((x, y), (landmark_x, landmark_y))
    
    # calculate bearing relative to the robot's orientation
    absolute_bearing = np.arctan2(landmark_y - y, landmark_x - x)
    relative_bearing = absolute_bearing - theta
    
    return np.array([r, relative_bearing])

def test_measurement_model(): 
    ldata = get_landmark_data()
    sample_run = {63:(2, 3, 0), 9:(0, 3, 0), 54:(1, -2, 0)}  #converted subjects to barcodes

    for subject_number, pose in sample_run.items():
        r, bearing = measurement_model(pose, subject_number, ldata)
        print(f"Robot pose: {pose}, Distance: {np.round(r, 3)} meters, Bearing: {np.round(bearing, 3)} radians")


def main(): 
     test_measurement_model()
     
if __name__ == '__main__':
    main()
