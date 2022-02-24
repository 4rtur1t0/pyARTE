#!/usr/bin/env python
# encoding: utf-8
"""
The orientation class
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.tools import rot2quaternion
from artelib import euler, quaternion, homogeneousmatrix


class RotationMatrix():

    def __init__(self, *args):
        orientation = args[0]
        if isinstance(orientation, np.ndarray):
            self.array = orientation
            self.array = self.array[0:3, 0:3]
        elif isinstance(orientation, euler.Euler):
            self.array = orientation.R()
        elif isinstance(orientation, quaternion.Quaternion):
            self.array = orientation.R()
        elif isinstance(orientation, RotationMatrix):
            self.array = orientation.toarray()
        else:
            raise Exception

    def __str__(self):
        return str(self.array)

    def toarray(self):
        return self.array

    def R(self):
        return self

    def Q(self):
        return quaternion.Quaternion(rot2quaternion(self.array))

    def homogeneous(self):
        return homogeneousmatrix.HomogeneousMatrix(np.zeros(3), self)