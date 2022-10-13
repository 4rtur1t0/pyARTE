#!/usr/bin/env python
# encoding: utf-8
"""
The HomogeneousMatrix class
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.tools import rot2quaternion, buildT
from artelib import quaternion, rotationmatrix, euler


class HomogeneousMatrix():
    def __init__(self, *args):
        if len(args) == 1:
            if isinstance(args[0], HomogeneousMatrix):
                self.array = args[0].toarray()
            elif isinstance(args[0], np.ndarray):
                self.array = args[0]
            else:
                self.array = np.array(args[0])
        if len(args) == 2:
            position = np.array(args[0])
            orientation = args[1]
            if isinstance(orientation, euler.Euler):
                array = buildT(position, orientation)
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

    def print_nice(self):
        temp_array = self.array
        th = 0.01
        idx = np.abs(temp_array) < th
        temp_array[idx] = 0
        print(np.array_str(self.array, precision=1, suppress_small=True))

    def inv(self):
        return HomogeneousMatrix(np.linalg.inv(self.array))

    def Q(self):
        return quaternion.Quaternion(rot2quaternion(self.array))

    def R(self):
        return rotationmatrix.RotationMatrix(self.array[0:3, 0:3])

    def pos(self):
        return self.array[0:3, 3]

    def __mul__(self, other):
        T = np.dot(self.array, other.array)
        return HomogeneousMatrix(T)

    def __add__(self, other):
        T = self.array+other.array
        return HomogeneousMatrix(T)

    def __sub__(self, other):
        T = self.array-other.array
        return HomogeneousMatrix(T)

    def __getitem__(self, item):
        return self.array[item[0], item[1]]




