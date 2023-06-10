import rclpy
import cv2
import numpy as np
import pickle

from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

from skspatial.objects import Plane, Points, LineSegment, Vector, Line

class CharucoDataNode(Node):

    def __init__(self):
        super().__init__('camera_charuco_ex_calib')
        self.get_logger().info("Waiting on MoCap and charuco data...")
        self.board_marker_sub_ = self.create_subscription(
            Marker,
            '/board/markers',
            self.marker_callback,
            10
        )
        
        self.image_sub_ = self.create_subscription(
            Image,
            '/camera0/image_raw',
            self.image_callback,
            10
        )

        self.cam_mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            '/camera1/pose',
            self.pose_callback,
            10
        )

        self.bridge = CvBridge()

        n_cols = 6
        n_rows = 8
        dim = 0.142
        marker_size = 0.107

        # aruco_dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        # aruco_parameters =  cv2.aruco.DetectorParameters_create()
        # self.detector = cv2.aruco.ArucoDetector(aruco_dictionary, aruco_parameters)
        # self.board = cv2.aruco.CharucoBoard((n_cols, n_rows), dim, marker_size, aruco_dictionary)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.board = cv2.aruco.CharucoBoard_create(n_cols, n_rows, dim, marker_size, self.aruco_dict)

        self.calib_data_count = 0
        self.max_data_count = 100
        self.markers = []
        self.cam_mocap_pose = []
        self.calib_data = {"ids": [], 
                           "corners": [], 
                           "markers": [], 
                           "cam_mocap_pose": []}
    
    def pose_callback(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.cam_mocap_pose = np.array([x,y,z,qw,qx,qy,qz])

    def marker_callback(self, msg):
        points = msg.points
        pos_list = []
        for p in points:
            pos_list.append([p.x, p.y, p.z])
        pos_list = np.array(pos_list)
        self.markers = pos_list
        # self.get_logger().info(f'Board maker pos:\n{pos_list}\n')

    def image_callback(self, msg):
        # self.get_logger().info("getting image...")
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        (corners, ids, rejected) = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
        # corners, ids, rejectedCandidates = self.detector.detectMarkers(img)

        # SUB PIXEL DETECTION
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.0001)
        for corner in corners:
            cv2.cornerSubPix(gray, corner, winSize = (3,3), zeroZone = (-1,-1), criteria = criteria)

        # frame_markers = cv2.aruco.drawDetectedMarkers(img.copy(), corners, ids)
        # frame_corners = img.copy()

        if ids is not None:
            num_corners, square_corners, ids = cv2.aruco.interpolateCornersCharuco(corners, ids, img, self.board)
            if num_corners >= 4:
                self.calib_data_count += 1
                self.calib_data["ids"].append(np.squeeze(ids))
                self.calib_data["corners"].append(np.squeeze(square_corners))
                self.calib_data["markers"].append(self.markers)
                self.calib_data["cam_mocap_pose"].append(self.cam_mocap_pose)
                self.get_logger().info(f"Data added, {self.calib_data_count}")
            frame_corners = cv2.aruco.drawDetectedCornersCharuco(img, square_corners, ids)
            # self.get_logger().info(f"{square_corners}")
            cv2.imshow("camera view", frame_corners)
        
        keyboard = cv2.waitKey(1)
        if keyboard == ord('q') or keyboard == 27:
            exit(0)

        if self.calib_data_count == self.max_data_count:
            # self.destroy_node()
            rclpy.shutdown()
            return
        
    def onshutdown(self):
        cv2.destroyAllWindows()
        # print(self.calib_data)
        with open("calib_data.pkl", "wb") as fs:
            pickle.dump(self.calib_data, fs)


def main(args=None):
    rclpy.init(args=args)

    node = CharucoDataNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.onshutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()