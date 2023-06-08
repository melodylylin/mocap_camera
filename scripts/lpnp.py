# from util import *
from lienp.SE3 import SE3
from lienp.base import wedge3

import numpy as np

R_model2cam = np.array([
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 0],        
])

# SE2, transform image coordiantes to match the book
SE2_pix2image = np.array([
    [-1, 0,  1600],
    [0, -1, 1200],
    [0, 0,  1],
])

def transform_intrinsic(K_opt):
    """
    K_opt should be (3,3) camera instrinsic matrix,
    after applying undistortion from cv2.getOptimalNewCameraMatrix()
    """
    K_ = SE2_pix2image @ K_opt @ R_model2cam
    return K_

def dot_circ(p_h):
    """
    p_h is the homogeneous coordinate
    p_h = [epsilon eta]^T
    where epsilon is in R^3 and eta is the scalar factor
    """
    e = p_h[0:3]
    s = p_h[3]
    X = np.block([
        [s * np.eye(3), -wedge3(e)],
        [np.zeros(6)]
    ])
    return X

def circ_circ(p_h):
    e = p_h[0:3]
    s = p_h[3]
    X = np.block([
        [np.zeros((3,3)), e.reshape(3,1)],
        [-wedge3(e), np.zeros((3,1))]
    ])

    return X

def get_V(K_inv, pix_h):
    """
    Projection matrix onto the pixel ray
    """
    bc = K_inv @ pix_h
    v = bc.reshape((3,1)) @ bc.reshape((1,3)) / (bc.T @ bc) # projection onto a line/ray 
    V = np.block([[v, np.zeros((3,1))],[np.zeros((1,3)), 1]]) # blocking the matrix to work with SE3 transformation matrix
    return V

def get_Vz_list(V_list, z):
    Vzdot_list = []
    Vz_list = []
    for Vi, zi in zip(V_list, z.T):
        Vz_i = (Vi - np.eye(4)) @ zi
        Vzdot_i = (Vi - np.eye(4)) @ dot_circ(zi)
        Vzdot_list.append(Vzdot_i)
        Vz_list.append(Vz_i)
    return Vz_list, Vzdot_list

def get_A(Vz_list, Vzdot_list):
    A = np.zeros((6,1))
    for Vz_i, Vzdot_i in zip(Vz_list, Vzdot_list):
        # print("==========")
        # print(Vi)
        # print(zi)
        A += (Vzdot_i.T @ Vz_i).reshape((6,1))
    return A

def get_B(Vzdot_list):
    B = np.zeros((6,6))
    for Vzdot_i in Vzdot_list:
        B += Vzdot_i.T @ Vzdot_i
    return B

def solve_lpnp(image_points, object_points, K_opt):
    """
    Returns optimal transformation of the camera pose and residual
    image_points: (N,2)
    object_points: (N,3)
    """
    assert image_points.shape[0] >= 4 and image_points.shape[1] == 2
    assert object_points.shape[0] >= 4 and object_points.shape[1] == 3
    assert image_points.shape[0] == object_points.shape[0]
    n_pairs = image_points.shape[0]
    K_ = transform_intrinsic(K_opt)
    K_t = np.block([
            [K_, np.zeros((3,1))],
        ])
    K_inv = np.linalg.inv(K_)
    image_points = np.hstack([image_points, np.ones((n_pairs,1))])
    object_points = np.hstack([object_points, np.ones((n_pairs,1))]).T

    V_list = [get_V(K_inv, p) for p in image_points]
    T_op = SE3(param=np.zeros(6))
    for i in range(10):
        z = T_op @ object_points
        Vz_list, Vzdot_list = get_Vz_list(V_list, z)
        A = get_A(Vz_list, Vzdot_list)
        B = get_B(Vzdot_list)
        ep = -np.linalg.inv(B) @ A
        T_op = SE3.exp(ep) @ T_op

    return T_op, ep
    #     print(f"error: {np.squeeze(ep)}")
    # print(f"True: {pose_true}")
    # print(f"Estimated: {T_op.get_param()}")
    # print(f"Absolute error: {T_op.get_param()- pose_true}")