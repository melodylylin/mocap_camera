import numpy as np
import cv2
import glob
import os
import yaml
import json
import matplotlib.pyplot as plt

usr_dir = os.path.expanduser('~')
data_dir = f'{usr_dir}/data/record1'
filepath = f'{usr_dir}/data/record1/*.jpg'
frame_name = glob.glob(filepath)
frame_name = np.sort(frame_name)

info_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/camera_info/"
with open(f'{info_dir}/camera_0.yaml', 'r') as fs:
    cam_info = yaml.safe_load(fs)
K = np.array(cam_info["camera_matrix"]["data"])
cam_dist = np.array(cam_info["distortion_coefficients"]["data"])
K_opt, dist_valid_roi = cv2.getOptimalNewCameraMatrix(K.reshape(3,3), cam_dist, (1600,1200), 1, (1600, 1200))

# undistort_img = cv2.undistort(img, K.reshape(3,3), cam_dist, None, newCameraMatrix=K_opt)
# undistort_points = cv2.undistortPoints(np.expand_dims(points, axis=1), K.reshape(3,3), cam_dist, P=K_opt)

# Initialize parameter setting using cv2.SimpleBlobDetector
# Set our filtering parameters
params = cv2.SimpleBlobDetector_Params()

params.minThreshold = 100
params.maxThreshold = 255

params.filterByColor = True
params.blobColor = 255

# Set Area filtering parameters
params.filterByArea = True
params.minArea = 20
params.maxArea = 150
  
# Set Circularity filtering parameters
params.filterByCircularity = True 
params.minCircularity = 0.5
  
# Set Convexity filtering parameters
params.filterByConvexity = True
params.minConvexity = 0.8
      
# Set inertia filtering parameters
params.filterByInertia = True
params.minInertiaRatio = 0.2
  
# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)

num_blobs = []
blobs_dict = {}

def find_blob(frame, detector, show_blob=False, save_blob=False, frame_idx=0):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    keypoints = detector.detect(frame)
    # Draw blobs
    blank = np.zeros((1, 1))
    blobs = cv2.drawKeypoints(frame, keypoints, blank, (100, 255, 200),
                            cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    
    # blobs = cv2.undistort(blobs, K.reshape(3,3), cam_dist, None, newCameraMatrix=K_opt)
    num_blobs.append(len(keypoints))
    text = "Number of Circular Blobs: " + str(len(keypoints))
    cv2.putText(blobs, text, (20, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2)
    

    # Show blobs
    if show_blob:
        cv2.imshow("Filtering Circular Blobs Only", blobs)
        if num_blobs == 0:
            keyboard = cv2.waitKey(0)
        else:
            keyboard = cv2.waitKey(1)
        if keyboard == 'q' or keyboard == 27:
            exit(0)
    if save_blob:
        blobs = np.array([kp.pt for kp in keypoints], dtype=float)
        blobs_dict[frame_idx] = blobs.tolist()

# frame = cv2.imread(f'{data_dir}/frame_{500}.jpg')
# find_blob(frame, detector, save_blob=True)

for i in range(500, 1200):
    frame = cv2.imread(f'{data_dir}/frame_{i}.jpg')
    find_blob(frame, detector, save_blob=True, frame_idx=i-500)

# print(blobs_dict)
# with open('blobs.json', 'w') as fs:
#     json.dump(blobs_dict, fs)

# num_blobs = np.array(num_blobs)
# plt.figure()
# plt.hist(num_blobs)
# plt.grid()
# plt.show()