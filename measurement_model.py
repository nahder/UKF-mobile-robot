import numpy as np 
from motion_testing import read_dat_file 

# Test your measurement model on predicting the range and heading of the following landmarks when the robot
# is located at the associate positions. Report your results. (Remember, you have access to the ground truth
# landmark positions.

#read in barcode data, associate barcode with subject number (dict)\
def get_landmark_data(): 
    landmark_ground_truth = read_dat_file('datasets/ds1/ds1_Landmark_Groundtruth.dat')

    landmark_dict = {}

    for row in landmark_ground_truth: 
        landmark_dict[row[0]]= (row[1],row[2]) #key: subject number, value: landmark (x,y)

    return landmark_dict 


#input: current position
#output: distances to all landmarks
def measurement_model(cur_pose,subject_number,landmark_data): 

    x,y,theta = cur_pose 
    
    pos_landmark = landmark_data[subject_number] 

    r = distance((x,y),pos_landmark) 
    
    bearing = np.arctan2(pos_landmark[1]-y, pos_landmark[0]-x)

    return r,bearing 



    
    




def distance(p1,p2): 
    return np.linalg.norm(np.array(p1)-np.array(p2))

def main(): 
   ldata = get_landmark_data()
   measurement_model((1,1,1),6,ldata)
   sample_run = {6:(2,3,0), 13:(0,3,0), 17:(1,-2,0)} 

   for sample in sample_run.items():
        r,bearing = measurement_model(sample[1],sample[0],ldata) 
        print(r,bearing)


    

   


if __name__ == "__main__": 
    main() 