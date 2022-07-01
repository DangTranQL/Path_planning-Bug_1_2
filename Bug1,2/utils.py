import cv2
import numpy as np
import math

def vec2hat(x):
    return np.array([[  0,     -x[2][0],   x[1][0]],
                     [ x[2][0],     0,     -x[0][0]],
                     [-x[1][0],  x[0][0],   0]])

def cvdata2transmtx(rvec,tvec):
    R_temp = cv2.Rodrigues(rvec)[0]
    p_temp = tvec.reshape(-1,1)
    R = R_temp.T
    p = -R.dot(p_temp)
    g = np.vstack((np.hstack((R,p)), [0, 0, 0 ,1]))
    return g, R, p

def cvdata2transmtx2(rvec,tvec):
    # TODO: Find the transformation matrix from the camera to the marker.
    R = cv2.Rodrigues(rvec)[0]
    p = tvec.reshape(-1,1)
    g = np.vstack((np.hstack((R,p)),np.array([0,0,0,1])))
    return g, R, p
def transmtx2twist(g):
    R = g[0:3,0:3]
    p = g[0:3,3]
    rot_exp_coord = cv2.Rodrigues(R)[0]
    th = np.linalg.norm(rot_exp_coord)
    w = rot_exp_coord/th
    v = np.linalg.inv((np.identity(3)-R).dot(vec2hat(w))+w.dot(w.T)*th).dot(p).reshape(-1,1)
    return v, w, th

def twist2screw(v,w,th):
    q = np.cross(w.reshape(-1),v.reshape(-1)).reshape(-1,1)
    h = w.T.dot(v)
    u = w
    M = th
    return q, h, u, M

def distance(xdiff,zdiff):
    dist = math.sqrt(xdiff**2 + zdiff**2)
    return dist