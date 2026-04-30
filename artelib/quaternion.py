#!/usr/bin/env python
# encoding: utf-8
"""
The Quaternion orientation class

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib import euler, rotationmatrix, homogeneousmatrix


class Quaternion():
    def __init__(self, array):
        self.array = np.array(array)

    def R(self):
        return rotationmatrix.RotationMatrix(quaternion2rot(self.array))

    def homogeneous(self):
        return homogeneousmatrix.HomogeneousMatrix(np.zeros(3), self.R())

    def Euler(self):
        """
        Convert Quaternion to Euler angles XYZ
        """
        return euler.Euler(q2euler(self.array)[0]), euler.Euler(q2euler(self.array)[1])

    def Q(self):
        return self

    def __str__(self):
        return str(self.array)

    def __getitem__(self, item):
        return self.array[item[0]]

    def toarray(self):
        return self.array

    def __add__(self, Q):
        return Quaternion(self.array + Q.toarray())

    def __sub__(self, Q):
        return Quaternion(self.array - Q.toarray())

    def dot(self, Q):
        """
        quaternion dot product
        """
        return np.dot(self.array, Q.toarray())

    def qconj(self):
        """
        quaternion conjugate
        """
        q0 = self.array[0]
        qv = -self.array[1:4]
        return Quaternion(np.hstack(([q0], qv)))

    def __mul__(self, Q):
        """
        quaternion product
        """
        if isinstance(Q, Quaternion):
            q1 = self.array
            q2 = Q.toarray()
            e0 = q1[0]*q2[0]-np.dot(q1[1:4], q2[1:4])
            e1 = np.dot(q1[0], q2[1:4]) + np.dot(q2[0], q1[1:4]) + np.cross(q1[1:4], q2[1:4])
            q = np.hstack(([e0], e1))
            return Quaternion(q)
        # this assumes that the rightmost element is a float
        elif isinstance(Q, int) or isinstance(Q, float):
            s = Q # scalar
            # q = self.array
            q = np.dot(s, self.array)
            return Quaternion(q)
        else:
            raise Exception('Quaternion product does not support the leftmost operand')

    def __truediv__(self, Q):
        """
        Division of quaternion by constant
        """
        q = self.array / Q
        return Quaternion(q)

    def slerp(self, Q2, t):
        """
        Interpolates between quaternions Q1 and Q2, given a fraction t in [0, 1].
        Caution: sign in the distance cth must be taken into account.
        """
        Q1 = self
        # caution using built-in class Quaternion  dot product
        cth = Q1.dot(Q2)
        # caution, saturate cos(th)
        cth = np.clip(cth, -1.0, 1.0)
        if cth < 0:
            cth = -cth
            Q1 = Q1 * (-1)

        th = np.arccos(cth)
        if np.abs(th) == 0:
            return Q1
        sth = np.sin(th)
        a = np.sin((1 - t) * th) / sth
        b = np.sin(t * th) / sth
        Q = Q1 * a + Q2 * b
        return Q

    def angle_between(self, Q2):
        """
        Computes the shortest angular distance (swept angle)
        between two unit quaternions.
        Returns angle in degrees.
        """
        q1 = self.toarray()
        q2 = Q2.toarray()
        # 1. Ensure they are unit quaternions
        q1 = q1 / np.linalg.norm(q1)
        q2 = q2 / np.linalg.norm(q2)

        # 2. Compute the dot product
        dot = np.dot(q1, q2)

        # 3. Take the absolute value to handle double cover
        # This ensures q and -q result in the same orientation
        abs_dot = np.abs(dot)

        # 4. Clamp the value to [0.0, 1.0] to prevent floating point
        # precision errors from making acos return NaN
        abs_dot = np.clip(abs_dot, 0.0, 1.0)

        # 5. Calculate the angle
        # The formula is theta = 2 * acos(|q1 . q2|)
        angle_rad = 2.0 * np.arccos(abs_dot)
        return angle_rad


def quaternion2rot(Q):
    qw = Q[0]
    qx = Q[1]
    qy = Q[2]
    qz = Q[3]
    R = np.eye(3)
    R[0, 0] = 1 - 2 * qy**2 - 2 * qz**2
    R[0, 1] = 2 * qx * qy - 2 * qz * qw
    R[0, 2] = 2 * qx * qz + 2 * qy * qw
    R[1, 0] = 2 * qx * qy + 2 * qz * qw
    R[1, 1] = 1 - 2*qx**2 - 2*qz**2
    R[1, 2] = 2 * qy * qz - 2 * qx * qw
    R[2, 0] = 2 * qx * qz - 2 * qy * qw
    R[2, 1] = 2 * qy * qz + 2 * qx * qw
    R[2, 2] = 1 - 2 * qx**2 - 2 * qy**2
    return R


def q2euler(Q):
    R = quaternion2rot(Q)
    abg = R.euler()
    # abg = rot2euler(R)
    return abg
