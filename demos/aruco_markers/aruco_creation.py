#!/usr/bin/env python
# encoding: utf-8
"""
   aruco creation
   Creates a dictionary of ARUCO MARKERS.
   The same dictionary (i.e. cv2.aruco.DICT_6X6_250) should be used
   in the camera.detect_arucos() method.

   A border is added to every ARUCO so that it can be stitched to any
   surface on Coppelia.
"""
from cv2 import aruco
import cv2
import matplotlib.pyplot as plt


def create_arucos():
    # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    plt.figure()

    top, bottom, left, right = 150, 150, 150, 150
    borderType = cv2.BORDER_CONSTANT
    value = [255, 255, 255]
    dst = None
    n = 30
    for i in range(n):
        img = aruco.generateImageMarker(aruco_dict, i, 700)
        img_border = cv2.copyMakeBorder(img, top, bottom, left, right,
                                  borderType, dst, value)

        cv2.imshow('borders', img_border)
        cv2.waitKey(500)
        cv2.imwrite('aruco' + str(i) + '.png', img_border)


if __name__ == "__main__":
    create_arucos()


