#!/usr/bin/env python
# encoding: utf-8
"""
   aruco DETECTION
   Just a demo that shows how to detect arucos and compute its pose.

   The estimated pose is
"""
import numpy as np
import cv2
import json
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from artelib.homogeneousmatrix import HomogeneousMatrix

CALIB_FILE = 'camera_calib.json'         # default camera calibration file
ARUCO_SIZE = 0.078 # in meters


def detect_arucos():
    # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        with open(CALIB_FILE) as file:
            data = json.load(file)
    except:
        print('Camera Calibration File not valid')
        exit()

    cameraMatrix = np.array(data['camera_matrix'])
    distCoeffs = np.array(data['distortion_coefficients'])

    print(f"{cameraMatrix=}")
    print(f"{distCoeffs=}")

    gray_image = cv2.imread('aruco_test.png')
    # cv2.imshow('WINDOW', gray_image)
    # cv2.waitKey(0)
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_image, aruco_dict)
    dispimage = cv2.aruco.drawDetectedMarkers(gray_image, corners, ids, borderColor=(0, 0, 255))

    # display corner order (Board file)
    for item in corners:
        for i in range(4):
            cv2.putText(dispimage, str(i), item[0, i].astype(int), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255, 0, 0), 1,
                       cv2.LINE_AA)
    cv2.imshow('WINDOW', dispimage)
    cv2.waitKey(0)
    # Calculate POSE
    if len(corners) > 0:
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, ARUCO_SIZE, cameraMatrix, distCoeffs)
        # Draw axis for each marker
        for i in range(len(rvecs)):
            dispimage = cv2.drawFrameAxes(dispimage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], length=50.0, thickness=2)
    # compute homogeneous matrices
    if len(rvecs) > 0:
        translation = Vector(tvecs[0][0])
        rotation, _ = cv2.Rodrigues(rvecs[0][0])
        rotation = RotationMatrix(rotation)
        Trel = HomogeneousMatrix(translation,
                                 rotation)
        print('ARUCO MARKER DETECTED! The transformation Tca (camera-ARUCO) is:')
        Trel.print_nice()
    else:
        print('NO ARUCO DETECTED!')
    # -----------------------------------------
    # Put your visualization code here
    # -----------------------------------------
    cv2.imshow('WINDOW', dispimage)
    cv2.waitKey(0)
    print('end')


if __name__ == "__main__":
    detect_arucos()


