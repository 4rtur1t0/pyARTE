#!/usr/bin/env python
# encoding: utf-8
"""
The HomogeneousMatrix class
@Authors: Arturo Gil
@Time: April 2023
"""
import numpy as np
# from artelib.euler import Euler
from artelib.tools import rot2quaternion, buildT
from artelib import quaternion, rotationmatrix, euler, vector
import matplotlib.pyplot as plt


class HomogeneousMatrix():
    def __init__(self, *args):
        if len(args) == 0:
            self.array = np.eye(4)
        elif len(args) == 1:
            if isinstance(args[0], HomogeneousMatrix):
                self.array = args[0].toarray()
            elif isinstance(args[0], np.ndarray):
                self.array = args[0]
            elif isinstance(args[0], list):
                self.array = np.array(args[0])
            else:
                self.array = np.array(args[0])
        elif len(args) == 2:
            position = args[0]
            orientation = args[1]
            if isinstance(position, list):
                position = np.array(position)
            elif isinstance(position, vector.Vector):
                position = np.array(position.array)

            if isinstance(orientation, euler.Euler):
                array = buildT(position, orientation)
            elif isinstance(orientation, list):
                array = buildT(position, euler.Euler(orientation))
            elif isinstance(orientation, quaternion.Quaternion):
                array = buildT(position, orientation)
            elif isinstance(orientation, rotationmatrix.RotationMatrix):
                array = buildT(position, orientation)
            else:
                raise Exception
            self.array = array

    def __str__(self):
        return str(self.array)

    def toarray(self):
        return self.array

    def print_nice(self, precision=5):
        temp_array = self.array
        th = 0.0001
        idx = np.abs(temp_array) < th
        temp_array[idx] = 0
        print(np.array_str(self.array, precision=precision, suppress_small=True))

    def print(self):
        self.print_nice()

    def inv(self):
        return HomogeneousMatrix(np.linalg.inv(self.array))

    def Q(self):
        return quaternion.Quaternion(rot2quaternion(self.array))

    def R(self):
        return rotationmatrix.RotationMatrix(self.array[0:3, 0:3])

    def euler(self):
        return self.R().euler()[0], self.R().euler()[1],

    def pos(self):
        return self.array[0:3, 3]

    def __mul__(self, other):
        if isinstance(other, HomogeneousMatrix):
            T = np.dot(self.array, other.array)
            return HomogeneousMatrix(T)
        elif isinstance(other, vector.Vector):
            u = np.dot(self.array, other.array)
            return vector.Vector(u)

    def __add__(self, other):
        T = self.array+other.array
        return HomogeneousMatrix(T)

    def __sub__(self, other):
        T = self.array-other.array
        return HomogeneousMatrix(T)

    def __getitem__(self, item):
        return self.array[item[0], item[1]]

    def t2v(self, n=2):
        # converting from SE(2)
        if n == 2:
            tx = self.array[0, 3]
            ty = self.array[1, 3]
            th = np.arctan2(self.array[1, 0], self.array[0, 0])
            return np.array([tx, ty, th])
        else:
            tx = self.array[0, 3]
            ty = self.array[1, 3]
            tz = self.array[2, 3]
            th = self.Q().Euler().abg
            return np.array([tx, ty, tz, th[0], th[1], th[2]])

    def plot(self, title='Homogeneous transformation', block=True):
        """
        Plot a rotation and translation using matplotlib's quiver method
        """
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        # first drawing the "-" . Next drawing two lines for each head ">"
        # colors = ['red', 'green', 'blue', 'red', 'red', 'green', 'green', 'blue', 'blue']
        ax.view_init(15, 35)
        # plot identity axes at (0, 0, 0)
        # identity = np.eye(3)
        pos = self.pos()
        # plot identity axis
        ax.quiver(0, 0, 0, 1, 0, 0, linestyle='dashed', color='red', linewidth=3)
        ax.quiver(0, 0, 0, 0, 1, 0, linestyle='dashed', color='green', linewidth=3)
        ax.quiver(0, 0, 0, 0, 0, 1, linestyle='dashed', color='blue', linewidth=3)

        # plot rotated axes
        # axis X
        ax.quiver(pos[0], pos[1], pos[2], self.array[0, 0], self.array[1, 0], self.array[2, 0], color='red',
                  linewidth=3)
        # axis y
        ax.quiver(pos[0], pos[1], pos[2], self.array[0, 1], self.array[1, 1], self.array[2, 1], color='green',
                  linewidth=3)
        # axis Z
        ax.quiver(pos[0], pos[1], pos[2], self.array[0, 2], self.array[1, 2], self.array[2, 2], color='blue',
                  linewidth=3)

        ax.set_xlim([min(-pos[0], -1)-1, max(pos[0], 1)+1])
        ax.set_ylim([min(-pos[1], -1)-1, max(pos[1], 1)+1])
        ax.set_zlim([min(-pos[2], 1)-1, max(pos[2], 1)+1])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        plt.title(title)
        plt.show(block=block)





