import numpy as np
from skspatial.objects import Line, LineSegment
R_cv = np.array([
        [ 0,  0,  1],
        [-1,  0,  0],
        [ 0, -1,  0]
    ])
from lienp.SE3 import SE3
from lienp.SO3 import SO3
def draw_ref_frame(ax, T: SE3, scale=1, alpha=.5):
    c = T.c
    R = T.R
    l1 = Line(c, R[0,:])
    l2 = Line(c, R[1,:])
    l3 = Line(c, R[2,:])
    ax1 = LineSegment(c, l1.to_point(scale))
    ax2 = LineSegment(c, l2.to_point(scale))
    ax3 = LineSegment(c, l3.to_point(scale))
    ax1.plot_3d(ax, color='r', alpha=alpha)
    ax2.plot_3d(ax, color='g', alpha=alpha)
    ax3.plot_3d(ax, color='b', alpha=alpha)
    ax.set_aspect("equal")

def from_cvframe(rvec, tvec):
    R_cam = SO3.exp(rvec)
    return SE3(R= R_cv @ R_cam, c=-R_cam.T @ tvec.reshape(-1))

from lienp.SO3 import so3
def to_cvframe(T):
    rvec = so3.log(R_cv.T @ T.R)
    tvec = R_cv.T @ -T.R @ T.c
    return rvec, tvec

def get_rtvec(T):
    rvec = so3.log(T.R)
    tvec = T.t
    return rvec, tvec

def project_points(obj_pts, T, K_opt):
    """
    T is expressed in OpenCV convention
    """
    obj_pts_h = np.hstack([obj_pts, np.ones((obj_pts.shape[0],1))])

    # R_cv = np.array([
    #     [ 0,  0,  1],
    #     [-1,  0,  0],
    #     [ 0, -1,  0]
    # ])
    # R_ = R_cv.T @ T.R
    # t_ = R_cv.T @ T.t
    # obj_pts_fcam = np.block([R_, t_.reshape(3,1)]) @ obj_pts_h.T
    obj_pts_fcam = T.M @ obj_pts_h.T
    obj_pts_fcam = obj_pts_fcam[0:3,:] / obj_pts_fcam[2,:]
    pix_pts = K_opt @ obj_pts_fcam
    return pix_pts