#!/usr/bin/env python3
import numpy as np
from lienp.SE3 import SE3
from lienp.SO3 import SO3, so3
import os, cv2, json, yaml

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from vision_msgs.msg import BoundingBox2D

from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from pathlib import Path
homedir = Path.home()

def get_3dbox(box_dim):
    vtx = (-1,1)
    box = []
    for ix in vtx:
        for iy in vtx:
            for iz in vtx:
                box.append(np.array([ix, iy, iz]) * box_dim/2)
    box = np.array(box)
    return box

class MocapCam(Node):
    def __init__(self, info_dir=None):
        super().__init__("MoCap_camera")
        self.bridge = CvBridge()
       
        self.declare_parameter('camera_name', 'camera3')
        self.declare_parameter('target_name', 'drone')
        self.declare_parameter('info_dir', f'{homedir}/.ros/camera_info')

        self.cam_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.target_name = self.get_parameter('target_name').get_parameter_value().string_value
        info_dir = self.get_parameter('info_dir').get_parameter_value().string_value
        
        self.target_size = np.array([0.3, 0.3, 0.20])
        self.target_3dbox = get_3dbox(self.target_size)

        with open(f'{info_dir}/{self.cam_name}.yaml', 'r') as fs:
            cam_info = yaml.safe_load(fs)
        if cam_info is not None:
            self.K = np.array(cam_info["camera_matrix"]["data"])
            self.cam_dist = np.array(cam_info["distortion_coefficients"]["data"])
            self.K_opt, dist_valid_roi = cv2.getOptimalNewCameraMatrix(self.K.reshape(3,3), self.cam_dist, (1600,1200), 1, (1600, 1200))
        
        with open(f"{info_dir}/{self.cam_name}_mocap_calib.json") as fs:
            calib_info = json.load(fs)
            self.R_rel = np.array(calib_info['R'])
            self.c_rel = np.array(calib_info['c'])
            self.q_rel = SO3.to_quat(self.R_rel)
            self.T_rel = SE3(R=self.R_rel, c=self.c_rel)

        self.delay = rclpy.time.Duration(seconds=0.1)

        self.T_mocap = SE3(np.zeros(6))
        self.T_cam = None

        # self.tf_buffer_ = Buffer()
        # self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.cam_mocap_pose_sub_ = self.create_subscription(
            PoseStamped,
            f"/{self.cam_name}/pose",
            self.cam_mocap_pose_cb,
            10,
        )

        if self.target_name is not None:
            self.target_pose = PoseStamped()
            self.target_markers = Marker()
            self.target_mocap_pose_sub_ = self.create_subscription(
                PoseStamped,
                f"/{self.target_name}/pose",
                self.target_mocap_pose_cb,
                10
            )

            self.target_marker_sub_ = self.create_subscription(
                Marker,
                f"/{self.target_name}/markers",
                self.target_marker_cb,
                10
            )

        self.img_sub_ = self.create_subscription(
            Image,
            f"{self.cam_name}/image_raw",
            self.image_cb,
            10,
        )

        self.cam_pose_pub_ = self.create_publisher(
            PoseStamped,
            f"{self.cam_name}/pose/calibrated",
            10
        )

        self.img_pub_ = self.create_publisher(
            Image,
            f"{self.cam_name}/image_rect/projection",
            10
        )

        self.bbox_pub_ = self.create_publisher(
            BoundingBox2D,
            f"{self.cam_name}/bbox",
            10
        )
    
    def cam_mocap_pose_cb(self, msg: PoseStamped):
        pos = msg.pose.position
        ori = msg.pose.orientation
        c = np.array([pos.x, pos.y, pos.z])
        q = np.array([ori.w, ori.x, ori.y, ori.z])
        R = SO3.from_quat(q).T
        self.T_mocap = SE3(R=R, c=c)
        self.T_cam = self.T_rel @ self.T_mocap

        # try:
        #     stamp = msg.header.stamp
        #     lookup_time_ = rclpy.time.Time.from_msg(stamp) - self.delay
        #     t = self.tf_buffer_.lookup_transform(
        #         'map', 
        #         f'{self.cam_name}/calibrated',
        #         lookup_time_)
        #     c = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
        #     q = np.array([t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z])
        #     R = SO3.from_quat(q).T
        #     self.T_cam = SE3(R=R, c=c)

        # except ValueError as e:
        #     self.get_logger().info(f"{e}")

        # except TransformException as ex:
        #     self.get_logger().info(
        #                 f'Could not transform map to {self.cam_name}/calibrated: {ex}')
        #     return

    def target_mocap_pose_cb(self, msg: PoseStamped):
        self.target_pose = msg
        # self.get_logger().info(f"fuck: {self.target_pose.pose.position}")
        # pos = msg.pose.position
        # ori = msg.pose.orientation
        # c = np.array([pos.x, pos.y, pos.z])
        # q = np.array([ori.w, ori.x, ori.y, ori.z])
        # R = SO3.from_quat(q).T
        # self.T_target = SE3(R=R, c=c)

    def target_marker_cb(self, msg: Marker):
        self.target_markers = msg

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        # img_proj = img.copy()
    
        ox = self.target_pose.pose.position.x
        oy = self.target_pose.pose.position.y
        oz = self.target_pose.pose.position.z
        obj_pt = np.array([ox, oy, oz])
        oqw = self.target_pose.pose.orientation.w
        oqx = self.target_pose.pose.orientation.x
        oqy = self.target_pose.pose.orientation.y
        oqz = self.target_pose.pose.orientation.z
        T_obj = SE3(R=SO3.from_quat(np.array([oqw,oqx,oqy,oqz])).T, c=obj_pt)
        
        markers_h = T_obj.inv @ np.hstack([self.target_3dbox, np.ones((8,1))]).T
        # markers = np.array([[p.x, p.y, p.z] for p in self.target_markers.points])
        # markers_h = np.hstack([markers, np.ones((markers.shape[0],1))]).T

        if self.T_cam is not None:
            # bounding box projection
            rvec, tvec = get_rtvec(self.T_cam)
            pix, _ = cv2.projectPoints(np.expand_dims(obj_pt, axis=0), rvec, tvec, self.K_opt, np.array([]))
            pix = np.squeeze(pix)

            box_msg = BoundingBox2D()
            box_msg.center.position.x = pix[0]
            box_msg.center.position.y = pix[1]
            box_msg.size_x = 20.
            box_msg.size_y = 20.
            self.bbox_pub_.publish(box_msg)
            img_rect = cv2.undistort(img, self.K.reshape(3,3), self.cam_dist, None, newCameraMatrix=self.K_opt)
            # img_rect = cv2.circle(img_rect, pix.astype(int), 50, (255,0,0), 1)

            pix, _ = cv2.projectPoints(markers_h[0:3,:].T, rvec, tvec, self.K_opt, np.array([]))
            pix = np.squeeze(pix)
            for p in pix:
                img_rect = cv2.circle(img_rect, p.astype(int), 3, (255,0,0), -1)

            box = cv2.boundingRect(pix.astype(int))
            cv2.rectangle(img_rect, box, 255, 2)

            img_rect = cv2.resize(img_rect, (800,600)) # reduce pic size to use less bandwidth when streaming
            img_msg = self.bridge.cv2_to_imgmsg(img_rect, encoding='mono8')
            img_msg.header.frame_id = msg.header.frame_id
            img_msg.header.stamp = msg.header.stamp
            self.img_pub_.publish(img_msg)
            
            # cv2.imshow("rectified", img_rect)
        # keyboard = cv2.waitKey(1)
        # if keyboard == ord('q') or keyboard == 27:
        #     exit(0)

def get_rtvec(T):
    rvec = so3.log(T.R)
    tvec = T.t
    return rvec, tvec

def project_points(obj_pts, T, K_opt):
    """
    T is expressed in OpenCV convention
    """
    obj_pts_h = np.hstack([obj_pts, np.ones((obj_pts.shape[0],1))])
    obj_pts_fcam = T.M @ obj_pts_h.T
    obj_pts_fcam = obj_pts_fcam[0:3,:] / obj_pts_fcam[2,:]
    pix_pts = K_opt @ obj_pts_fcam
    return pix_pts[0:2,:]

def main(args=None):
    rclpy.init(args=args)
    
    node = MocapCam()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.onshutdown()
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()