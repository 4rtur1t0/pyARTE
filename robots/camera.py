#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage a camera in Coppelia simulations (a vision sensor)

@Authors: Arturo Gil
@Time: April 2021

"""
from PIL import Image, ImageOps
import numpy as np
import cv2
from artelib.vector import Vector
from artelib.rotationmatrix import RotationMatrix
from artelib.homogeneousmatrix import HomogeneousMatrix


class Camera():
    def __init__(self, simulation, resolution=1200, fov_degrees=45.0):
        """
        Construct the camera object.
        wh: resolution of the image (must be square)
        fov: camera fov in degrees
        """
        self.simulation = simulation
        self.camera = None
        # image width and height
        self.wh = resolution
        self.fov = np.deg2rad(fov_degrees)
        self.fxy = self.wh / (2 * np.tan(self.fov / 2))

    def start(self, name='camera'):
        camera = self.simulation.sim.getObject(name)
        self.camera = camera

    def get_image(self):
        print('Capturing image of vision sensor ')
        # resolution, image = self.simulation.sim.getVisionSensorImg(self.camera, 0)
        image, resX, resY = self.simulation.sim.getVisionSensorCharImage(self.camera)
        image = np.frombuffer(image, dtype=np.uint8).reshape(resY, resX, 3)
        # return image in openCV format.
        # image = np.array(image, dtype=np.uint8)
        # image.resize([resolution[1], resolution[0], 3])
        # image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 0)
        image = cv2.flip(image, 0)
        # image.resize([resY, resX, 3])
        # cv2.imshow('', image)
        # cv2.waitKey(1)
        print('Image captured!')
        return image

    def get_mean_color(self):
        """
        Returns an [R, G, B] array.
        """
        image = self.get_image()
        mean_color = np.mean(image, axis=(0, 1))
        try:
            return mean_color / np.linalg.norm(mean_color)
        except:
            return mean_color

    def get_color_name(self):
        """
        Returns the mean color of the image as 'R', 'G' or 'B' using an Euclidean distance
        """
        # get the mean color of the image captured
        mean_color = self.get_mean_color()
        # define perfect colors as R, G, B. normalized
        colors = np.array([[1, 0, 0], [0., 1, 0], [0, 0, 1]])
        color_names = ['R', 'G', 'B']
        d = np.linalg.norm(colors - mean_color, axis=1)
        color_index = np.argmin(d)
        # return the closest color found
        color = color_names[color_index]
        print('Piece is: ', color)
        return color

    def save_image(self, filename):
        """
        Captures an image and saves it to filename.
        """
        image = self.get_image()
        img = Image.fromarray(image)
        # img = ImageOps.flip(img)
        print('Saving to file: ', filename)
        img.save(filename)
        # caution--> tell python to release memory now or a kill will be issued by the system!
        del img
        del image
        print('Image saved!')

    def detect_arucos(self, show=False, aruco_size=0.1):
        """
        Returns the transformation to the detected arucos
        """
        # ARUCO_SIZE = 0.078  # in meters, size of the ARUCO marker in simulation
        # CAUTION: this is true for the simulations in this particular library
        # ARUCO_SIZE = 0.07  # in meters, size of the ARUCO marker in simulation
        # ARUCO_SIZE = 0.0695  # in meters, size of the ARUCO marker in simulation
        gray_image = self.get_image()
        # gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        # The dictionary should be defined as the one used in demos/aruco_markers/aruco_creation.py
        # aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        # In this special case, the focal, width and height of the camera are computed based on Coppelia's parameters:
        # FOV and resolution
        cameraMatrix = np.array([[self.fxy, 0.0, (self.wh-1)/2],
                                 [0.0, self.fxy, (self.wh-1)/2],
                                 [0.0, 0.0, 1.0]])
        distCoeffs = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray_image, aruco_dict)
        if len(corners) > 0:
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                           aruco_size,
                                                                           cameraMatrix,
                                                                           distCoeffs)

        if show:
            cv2.imshow('aruco_detect', gray_image)
            cv2.waitKey(1000)
            dispimage = cv2.aruco.drawDetectedMarkers(gray_image, corners, ids, borderColor=(0, 0, 255))
            # display corner order (Board file)
            for item in corners:
                for i in range(4):
                    cv2.putText(dispimage, str(i), item[0, i].astype(int), cv2.FONT_HERSHEY_DUPLEX, 0.4, (255, 0, 0), 1,
                                cv2.LINE_AA)
            if len(corners) > 0:
                # Draw axis for each marker
                for i in range(len(rvecs)):
                    dispimage = cv2.drawFrameAxes(dispimage, cameraMatrix, distCoeffs, rvecs[i], tvecs[i],
                                                  length=0.2,
                                                  thickness=2)

            cv2.imshow('aruco_detect', dispimage)
            cv2.waitKey(1000)

        if len(corners) > 0:
            # compute homogeneous matrices
            tranformations = []
            for i in range(len(rvecs)):
                translation = Vector(tvecs[i][0])
                rotation, _ = cv2.Rodrigues(rvecs[i][0])
                rotation = RotationMatrix(rotation)
                Trel = HomogeneousMatrix(translation,
                                     rotation)
                tranformations.append(Trel)
            return ids, tranformations

        return None, None

    def detect_closer_aruco(self, show=False, aruco_size=0.1):
        """
        Returns the ARUCO marker closer to the camera frame
        """
        ids, transformations = self.detect_arucos(show=show, aruco_size=aruco_size)
        if ids is None:
            return None, None
        # find the ARUCO that is closer to the camera
        d = []
        for transformation in transformations:
            d.append(np.linalg.norm(transformation.pos()))
        closer_index = np.argmin(d)
        # Encontramos ahora la transformación total hasta todas la ARUCO más cercana
        # print('ID: ', ids[closer_index][0])
        id = ids[closer_index][0]
        # transformations[closer_index].print_nice()
        Tca = transformations[closer_index]
        return id, Tca
