#!/usr/bin/env python
# encoding: utf-8
"""
// Example program using OpenCV library
//      python >3.7 - OpenCV 4.5
// @file	e8.py
// @author Luis M. Jimenez
   Modified: Arturo Gil, January 2024
// @date 2022, 2023
//
// @brief Course: Computer Vision (1782)
// Dept. of Systems Engineering and Automation
// Automation, Robotics and Computer Vision Lab (ARVC)
// http://arvc.umh.es
// University Miguel Hernandez
//
// @note Description:
//	- This example captures images from a camera, shows them in a window and...
//  - Enables a handler for mouse events
//  - Saves window image on leftmouse+Shift click
//  - Saves window image on Space bar or F5 keystroke
//  - Calibrate camera
//  - Save calibration data to JSON
//
"""
# Import libraries
import cv2 as cv
import numpy as np
import os
import json

NO_DISTORTION = True
# Calibrate camera with stored image files (calibration pattern)
# Calibration pattern inner corners (cols Y-axis, rows X-axis)
patternSize = (9, 6)
# CAUTION: this is not the standard size. Please, bear in mind that the pattern may be resized when included in Coppelia
squareSize = 33.65       # square pattern side in mm

# 3D object points coordinates (x,y,z)
objp3D = np.zeros((patternSize[1], patternSize[0], 3), np.float32)
for x in range(patternSize[1]):
    for y in range(patternSize[0]):
        objp3D[x,y] = (x*squareSize, y*squareSize, 0)

objp3D = objp3D.reshape(-1, 3)   #  transform in a  row vector of (x,y,z) tuples

objpoints = []      # 3D point in real world space
imgpoints = []      # 2D points in image plane
num_patterns = 0    # number of detected patterns. We need at lest 3

# reading directory files
prefix = 'calib_pattern'
valid_extension = ('jpg', 'jpeg', 'png', 'tiff', 'tif', 'bmp', 'pgm')
directoryList = sorted(os.listdir("./captures"))
for file in directoryList:
    # Filter non image files
    if not file.startswith(prefix) or not file.endswith(valid_extension):
        continue

    print(f"Processing image file: {file}")
    image = cv.imread('./captures/' + file)
    gray_image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)  # transforms to gray level

    # cv.imshow('tt', gray_image)
    # cv.waitKey(0)

    # Calibration pattern inner corners (cols Y-axis, rows X-axis)
    patternWasFound, corners = cv.findChessboardCorners(gray_image, patternSize)

    if patternWasFound:
        num_patterns += 1
        # Iterative algorithm termination criteria
        termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv.cornerSubPix(gray_image, corners, winSize=(11, 11), zeroZone=(-1, -1), criteria=termCriteria)

        # add points to image/3Dobjet lists
        objpoints.append(objp3D)
        imgpoints.append(corners)

        # Draw and display the corners
        cv.drawChessboardCorners(image, patternSize, corners, patternWasFound)

        cv.imshow('WINDOW_CAMERA', image)     # Display the resulting frame
        key = cv.waitKey(1000)              # update image and wait 1 second

# end for file in  os.listdir("."):

if num_patterns >= 3:
    # imageSize:  (cols, rows)
    termCriteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cameraMatrix = np.array([])
    if NO_DISTORTION:
        distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        flags = cv.CALIB_FIX_K1+cv.CALIB_FIX_K2+ \
            cv.CALIB_FIX_K3 + cv.CALIB_FIX_K4 + \
            cv.CALIB_FIX_K5 + cv.CALIB_FIX_K6
        rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints,
                                                                     imageSize=gray_image.shape[::-1],
                                                                     cameraMatrix=None,
                                                                     distCoeffs=distCoeffs,
                                                                     flags= flags,
                                                                     criteria=termCriteria)
    else:
        rms, cameraMatrix, distCoeffs, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints,
                                                                        imageSize=gray_image.shape[::-1],
                                                                        cameraMatrix=None,
                                                                        distCoeffs=None,
                                                                        flags=0,
                                                                        criteria=termCriteria)
    print(f"RMS reprojection error: {rms}")

    # store calibration data in a JSON file
    with open('camera_calib.json', 'w') as file:
        json.dump({'camera_matrix': cameraMatrix.tolist(),
                   'distortion_coefficients': distCoeffs.tolist()}, file)

    # Reading calibration data from file
    try:
        with open('camera_calib.json') as file:
            data = json.load(file)

        camera_matrix = np.array(data['camera_matrix'])
        distortion_coefficients = np.array(data['distortion_coefficients'])

        print('Camera matrix:', camera_matrix)
        print('distortion_coefficients:', distortion_coefficients)

    except:
        print("File not valid")



