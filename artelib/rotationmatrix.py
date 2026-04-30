#!/usr/bin/env python
# encoding: utf-8
"""
The Rotation Matrix class
@Authors: Arturo Gil
@Time: July 2022
"""
import numpy as np
from artelib import euler, quaternion, homogeneousmatrix, vector
from artelib.tools import normalize_angle
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
        return quaternion.Quaternion(rot2quaternion(self))

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

    def __add__(self, other):
        """
        For debugging purposes. The sum of two rotation matrices IS NOT a rotation matrix
        """
        R = self.array+other.array
        return RotationMatrix(R)

    def __sub__(self, other):
        """
        For debugging purposes. The sum of two rotation matrices IS OBVIOUSLY NOT a rotation matrix
        """
        R = self.array-other.array
        return RotationMatrix(R)

    def angle_between(self, R2):
        """
        Computes the spherical angle between two rotation matrices
        """
        # convert to quaternions
        Q1 = self.Q()
        Q2 = R2.Q()
        # compute the angular speed w that rotates from Q1 to Q2
        a = Q1.angle_between(Q2)
        return a

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


def rot2quaternion(R):
    """
    Converts a 3x3 rotation matrix to a unit quaternion (w, x, y, z)
    using Shepperd's Algorithm for numerical stability.
    """
    R = np.array(R.toarray(), dtype=np.float64)
    tr = np.trace(R)

    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2  # S = 4 * qw
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2  # S = 4 * qx
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2  # S = 4 * qy
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2  # S = 4 * qz
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    # Return normalized quaternion to fix any tiny floating-point drift
    q = np.array([w, x, y, z])
    return q / np.linalg.norm(q)


def rot2euler(R):
    """
    Conversion from the rotation matrix R to Euler angles.
    The XYZ convention in mobile axes is assumed.
    """
    th = np.abs(np.abs(R[0, 2])-1.0)
    R[0, 2] = min(R[0, 2], 1)
    R[0, 2] = max(R[0, 2], -1)

    # caso no degenerado
    if th > 0.0001:
        beta1 = np.arcsin(R[0, 2])
        beta2 = np.pi - beta1
        s1 = np.sign(np.cos(beta1))
        s2 = np.sign(np.cos(beta2))
        alpha1 = np.arctan2(-s1*R[1][2], s1*R[2][2])
        gamma1 = np.arctan2(-s1*R[0][1], s1*R[0][0])
        alpha2 = np.arctan2(-s2*R[1][2], s2*R[2][2])
        gamma2 = np.arctan2(-s2*R[0][1], s2*R[0][0])
    # degenerate case
    else:
        print('CAUTION: rot2euler detected a degenerate solution when computing the Euler angles.')
        alpha1 = 0
        alpha2 = np.pi
        beta1 = np.arcsin(R[0, 2])
        if beta1 > 0:
            beta2 = np.pi/2
            gamma1 = np.arctan2(R[1][0], R[1][1])
            gamma2 = np.arctan2(R[1][0], R[1][1])-alpha2
        else:
            beta2 = -np.pi/2
            gamma1 = np.arctan2(-R[1][0], R[1][1])
            gamma2 = np.arctan2(-R[1][0], R[1][1])-alpha2
    # finally normalize to +-pi
    e1 = normalize_angle([alpha1, beta1, gamma1])
    e2 = normalize_angle([alpha2, beta2, gamma2])
    return e1, e2