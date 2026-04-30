#!/usr/bin/env python
# encoding: utf-8
"""
   aruco DETECTION
   Just a demo that shows how to detect arucos
"""
import numpy as np
import cv2
import json
CALIB_FILE = 'camera_calib.json'         # default camera calibration file


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
    print('end')


if __name__ == "__main__":
    detect_arucos()


