#!/usr/bin/env python
# encoding: utf-8
"""
The Rotation Matrix class
@Authors: Arturo Gil
@Time: July 2022
"""
import numpy as np
from artelib.tools import rot2quaternion, rot2euler
from artelib import euler, quaternion, homogeneousmatrix, vector
import matplotlib.pyplot as plt


class RotationMatrix():
    def __init__(self, *args):
        orientation = args[0]
        self.array = None
        # constructor from a np array
        if isinstance(orientation, np.ndarray):
            self.array = orientation
        elif isinstance(orientation, list):
            self.array = np.array(orientation)
            # self.array = self.array[0:3, 0:3]
        elif isinstance(orientation, int):
            self.array = np.eye(orientation)
        elif isinstance(orientation, euler.Euler):
            self.array = orientation.R()
        elif isinstance(orientation, quaternion.Quaternion):
            self.array = orientation.R()
        # copy constructor
        elif isinstance(orientation, RotationMatrix):
            self.array = orientation.toarray()
        else:
            raise Exception

    def __str__(self):
        return str(self.array)

    def __getitem__(self, item):
        return self.array[item[0], item[1]]

    def print_nice(self, precision=3):
        temp_array = self.array
        th = 0.01
        idx = np.abs(temp_array) < th
        temp_array[idx] = 0
        print(np.array_str(self.array, precision=precision, suppress_small=True))

    def print(self):
        self.print_nice()

    def toarray(self):
        return self.array

    def inv(self):
        return RotationMatrix(self.array.T)

    def T(self):
        """
        transpose, as in numpy
        """
        return RotationMatrix(self.array.T)

    def det(self):
        return np.linalg.det(self.array)

    def R(self):
        return self

    def Q(self):
        return quaternion.Quaternion(rot2quaternion(self.array))

    def euler(self):
        eul = rot2euler(self.array)
        return euler.Euler(eul[0]), euler.Euler(eul[1])

    def homogeneous(self):
        return homogeneousmatrix.HomogeneousMatrix(np.zeros(3), self)

    def __mul__(self, other):
        """
        Multiply matrices or multiply a matrix and a vector.
        """
        if isinstance(other, RotationMatrix):
            R = np.dot(self.array, other.array)
            return RotationMatrix(R)
        elif isinstance(other, vector.Vector):
            u = np.dot(self.array, other.array)
            return vector.Vector(u)

    def plot(self, title='Rotation Matrix', block=True):
        """
        Plot the rotation matrix as 2D or 3D vectors
        """
        n = self.array.shape[0]
        fig = plt.figure()
        if n == 2:
            # origin = np.array([[0, 0], [0, 0]])  # origin point
            plt.quiver((0, 0), (0, 0), self.array[0, :], self.array[1, :], color=['red', 'green'], angles='xy',
                       scale_units='xy', scale=1)
            plt.draw()
            plt.xlim([-1, 1])
            plt.ylim([-1, 1])
            plt.xlabel('X')
            plt.ylabel('Y')
        elif n == 3:
            ax = fig.add_subplot(projection='3d')
            # first drawing the "-" . Next drawing two lines for each head ">"
            colors = ['red', 'green', 'blue', 'red', 'red', 'green', 'green', 'blue', 'blue']
            ax.view_init(15, 35)
            # plot identity
            I = np.eye(3)
            ax.quiver(0, 0, 0, I[0, :], I[1, :], I[2, :], color=colors, linestyle='dashed', linewidth=3)
            ax.quiver(0, 0, 0, self.array[0, :], self.array[1, :], self.array[2, :], color=colors, linewidth=3)
            ax.set_xlim([-1, 1])
            ax.set_ylim([-1, 1])
            ax.set_zlim([-1, 1])
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.set_zlabel('Z')
        plt.title(title)
        plt.show(block=block)


def R2(theta):
    """
    A 2x2 rotation matrix.
    """
    R = np.array([[np.cos(theta), -np.sin(theta)],
                  [np.sin(theta), np.cos(theta)]])
    return RotationMatrix(R)


def Rx(theta):
    """
    A fundamental rotation along the X axis.
    """
    R = np.array([[1,       0,                    0],
                  [0, np.cos(theta), -np.sin(theta)],
                  [0, np.sin(theta), np.cos(theta)]])
    return RotationMatrix(R)


def Ry(theta):
    """
    A fundamental rotation along the Y axis.
    """
    R = np.array([[np.cos(theta), 0,    np.sin(theta)],
                  [0,             1,                0],
                  [-np.sin(theta),  0,     np.cos(theta)]])
    return RotationMatrix(R)


def Rz(theta):
    """
    A fundamental rotation along the Z axis.
    """
    R = np.array([[np.cos(theta), -np.sin(theta),  0],
                  [np.sin(theta),  np.cos(theta),  0],
                  [0,              0,        1]])
    return RotationMatrix(R)
