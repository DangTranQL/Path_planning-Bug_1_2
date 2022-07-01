import cv2
import numpy as np
import yaml
import utils
import math

# The different ArUco dictionaries built into the OpenCV library. 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_250)
aruco_params = cv2.aruco.DetectorParameters_create()
aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR

# Side length of the ArUco marker in meters 
marker_length = 0.05

# Calibration parameters yaml file
with open(r'calib_data.yaml') as file:
    calib_data = yaml.load(file, Loader=yaml.FullLoader)

mtx = np.asarray(calib_data["camera_matrix"])
dist = np.asarray(calib_data["distortion_coefficients"])

cap = cv2.VideoCapture(cv2.CAP_V4L)

goal_id = 1
helper1_id = 0
helper2_id = 2

print("Press q to save the transformation between Goal and Helper 1")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (len(corners)!=0):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i,_ in enumerate(rvecs):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc1 = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                elif ids[i] == helper1_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch1 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])[0]
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

print("Press q to save the transformation between Goal and Helper 2")
while cap.isOpened():
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
        if (len(corners)!=0):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, mtx, dist)
            cv2.aruco.drawDetectedMarkers(frame, corners)
            for i,_ in enumerate(rvecs):
                if ids[i] == goal_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_gc2 = utils.cvdata2transmtx(rvecs[i],tvecs[i])[0]
                elif ids[i] == helper2_id:
                    cv2.aruco.drawAxis(frame, mtx, dist, rvecs[i], tvecs[i], 0.05)
                    g_ch2 = utils.cvdata2transmtx2(rvecs[i],tvecs[i])[0]
        cv2.imshow('aruco',frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()

g_gh1 = g_gc1.dot(g_ch1)
print("Helper1 location x:{}, z:{}".format(g_gh1[0,3],g_gh1[2,3]))
g_gh2 = g_gc2.dot(g_ch2)
print("Helper2 location x:{}, z:{}".format(g_gh2[0,3],g_gh2[2,3]))

calib_data = {
    'camera_matrix':np.asarray(mtx).tolist(),
    'distortion_coefficients':np.asarray(dist).tolist(),
    'g_gh1':np.asarray(g_gh1).tolist(),
    'g_gh2':np.asarray(g_gh2).tolist()
}

with open(r'calib_data_bug1.yaml', 'w') as file:
    yaml.dump(calib_data, file)