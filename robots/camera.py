#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage a camera in Coppelia simulations (a vision sensor)

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
import time
import numpy as np
from PIL import Image, ImageOps


class Camera():
    def __init__(self, clientID):
        self.clientID = clientID
        self.camera = None

    def start(self, name='camera'):
        errorCode, camera = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_oneshot_wait)
        self.camera = camera

    def get_image(self):
        print('Capturing image of vision sensor ')
        errorCode, resolution, image = sim.simxGetVisionSensorImage(self.clientID, self.camera, 0,
                                                                    sim.simx_opmode_oneshot_wait)
        # return image in openCV format.
        image = np.array(image, dtype=np.uint8)
        image.resize([resolution[1], resolution[0], 3])
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