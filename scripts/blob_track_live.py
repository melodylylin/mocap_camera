import numpy as np
import cv2
from datetime import date, datetime
date_str = f"{date.today().strftime('%Y-%m-%d')}-{datetime.now().time().strftime('%I%M%S')}"


class BlobTracker():
    def __init__(self, detector) -> None:
        self.start_record = False
        self.detector = detector

    def find_blob(self, frame):
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)        
        keypoints = self.detector.detect(frame)
        # Draw blobs
        blank = np.zeros((1, 1))
        blobs = cv2.drawKeypoints(frame, keypoints, blank, (100, 255, 200),
                                cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        
        text = "Number of Circular Blobs: " + str(len(keypoints))
        cv2.putText(blobs, text, (20, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 100, 255), 2)
        
        # Show blobs
        cv2.imshow("Filtering Circular Blobs Only", blobs)
        keyboard = cv2.waitKey(1)
        # keyboard = cv2.waitKey(int(1/5 * 1000))
        if keyboard == ord('q') or keyboard == 27:
            print('Exiting...')
            exit(0)

        if keyboard == ord('s'):
            self.start_record = True
            print('Started recording...')
        elif keyboard == ord('d'):
            self.start_record = False
            print('Stopped recording...')

    def save_frame(self, frame, path):
        cv2.imwrite(path, frame)

    def open(self, device_id, fps=30, width=1600, height=1200):
        camera = cv2.VideoCapture(device_id, cv2.CAP_V4L)

        fps_ok = camera.set(cv2.CAP_PROP_FPS, fps)
        fourcc_ok = camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
        width_ok = camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        height_ok = camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if fps_ok and fourcc_ok and width_ok and height_ok:
            return camera
        else:
            print(fps_ok, fourcc_ok, width_ok, height_ok)
            return None


def main():
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

    blob_tracker = BlobTracker(detector)

    camera = blob_tracker.open(0)
    if camera is None:
        print('cannot open camera')
        exit(0)
   
    while True:
        ret, frame = camera.read()
        if frame is None:
            break

        if blob_tracker.start_record:
            i=0
            path = f'frame_{i}.jpg'
            blob_tracker.save_frame(frame, path)

        blob_tracker.find_blob(frame)

if __name__=='__main__':
    main()