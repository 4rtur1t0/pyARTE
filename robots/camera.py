#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage a camera in Coppelia simulations (a vision sensor)

@Authors: Arturo Gil
@Time: April 2021

"""
# import sim
# import time
import numpy as np
from PIL import Image, ImageOps
import cv2


class Camera():
    def __init__(self, simulation):
        self.simulation = simulation
        self.camera = None

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
        # sensorImage = np.array(image, dtype=np.uint8)
        # sensorImage.resize([resolution[1], resolution[0], 3])
        img = Image.fromarray(image)
        # img = ImageOps.flip(img)
        print('Saving to file: ', filename)
        img.save(filename)
        # caution--> tell python to release memory now or a kill will be issued by the system!
        del img
        del image
        print('Image saved!')