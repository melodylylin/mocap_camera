import numpy as np
import cv2
import matplotlib.pyplot as plt

n_cols = 6
n_rows = 8
dim = 0.142
marker_size = 0.107

# dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
# parameters =  cv2.aruco.DetectorParameters()
# detector = cv2.aruco.ArucoDetector(dictionary, parameters)
# board = cv2.aruco.CharucoBoard((n_cols, n_rows), dim, marker_size, dictionary)

aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
arucoParams = cv2.aruco.DetectorParameters_create()
board = cv2.aruco.CharucoBoard_create(n_cols, n_rows, dim, marker_size, aruco_dict)
# board_img = board.draw((2000,2000))

# plt.imsave("board.jpg", board_img)

def open(device_id, fps=30, width=1600, height=1200):
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

camera = open(0)
if camera is None:
    print("cannot open camera")
    exit(0)

while True:
    ret, frame = camera.read()
    if frame is None:
        break 
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)
    # corners, ids, rejectedCandidates = detector.detectMarkers(frame)

    # SUB PIXEL DETECTION
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
    for corner in corners:
        cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

    frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    frame_corners = frame.copy()

    if ids is not None:
        _, square_corners, ids = cv2.aruco.interpolateCornersCharuco(corners, ids, frame, board)
        frame_corners = cv2.aruco.drawDetectedCornersCharuco(frame.copy(), square_corners, ids)

    cv2.imshow("aruco", frame_markers)
    cv2.imshow("aruco_corner", frame_corners)
    keyboard = cv2.waitKey(30)
    if keyboard == ord('q') or keyboard == 27:
        exit(0)