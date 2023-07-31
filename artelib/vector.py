#!/usr/bin/env python
# encoding: utf-8
"""
A class to hold vectors.
This allows multiplication by the clases RotationMatrix and HomogeneousMatrix.
@Authors: Arturo Gil
@Time: July 2022
"""
import numpy as np
from artelib import homogeneousmatrix
import matplotlib.pyplot as plt


class Vector():
    def __init__(self, *args):
        vector = args[0]
        # constructor from a np array
        if isinstance(vector, np.ndarray):
            self.array = vector
        elif isinstance(vector, list):
            self.array = np.array(vector)
        else:
            raise Exception

    def __str__(self):
        return str(self.array)

    def toarray(self):
        return self.array

    def homogeneous(self):
        return homogeneousmatrix.HomogeneousMatrix(self.array, np.eye(3))

    def __add__(self, other):
        return Vector(self.array + other.array)

    def __sub__(self, other):
        return Vector(self.array - other.array)

    def __mul__(self, other):
        """
        scalar product
        """
        scalar = np.dot(self.array, other.array)
        return scalar

    def cross(self, other):
        return Vector(np.cross(self.array, other.array))

    def pos(self):
        return np.array(self.array)

    def T(self):
        """
        Transpose, as in numpy
        """
        return Vector(self.array.T)

    def plot(self, title='3D Vector', block=True):
        """
        Plot a Vector using matplotlib's quiver method
        """
        n = self.array.shape[0]
        fig = plt.figure()
        if n == 2:
            # origin = np.array([[0, 0], [0, 0]])  # origin point
            identity = np.eye(2)
            x_pos = 0
            y_pos = 0
            x_vector = self.array[0]
            y_vector = self.array[1]
            plt.quiver((0, 0), (0, 0), identity[0, :], identity[1, :], color=['red', 'green'], angles='xy',
                   scale_units='xy', scale=1)
            plt.quiver(x_pos, y_pos, x_vector, y_vector, angles='xy', scale_units='xy', scale=1)
            plt.draw()
            plt.xlim([min(-x_vector, -1), max(x_vector, 1)])
            plt.ylim([min(-y_vector, -1), max(y_vector, 1)])
            plt.xlabel('X')
            plt.ylabel('Y')
        # 3D or homogeneous
        elif n >= 3:
            ax = fig.add_subplot(projection='3d')
            # first drawing the "-" . Next drawing two lines for each head ">"
            colors = ['red', 'green', 'blue', 'red', 'red', 'green', 'green', 'blue', 'blue']
            ax.view_init(15, 35)
            # plot identity
            I = np.eye(3)
            ax.quiver(0, 0, 0, I[0, :], I[1, :], I[2, :], color=colors, linestyle='dashed', linewidth=3)
            # ax.quiver(0, 0, 0, self.array[0, :], self.array[1, :], self.array[2, :], color=colors, linewidth=3)
            x_pos = 0
            y_pos = 0
            z_pos = 0
            x_vector = self.array[0]
            y_vector = self.array[1]
            z_vector = self.array[2]
            plt.quiver(x_pos, y_pos, z_pos, x_vector, y_vector, z_vector)
            ax.set_xlim([min(-x_vector, -1), max(x_vector, 1)])
            ax.set_ylim([min(-y_vector, -1), max(y_vector, 1)])
            ax.set_zlim([min(-z_vector, -1), max(z_vector, 1)])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
        plt.title(title)
        plt.show(block=block)
