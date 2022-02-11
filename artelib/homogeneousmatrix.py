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

    def toarray(self):
        return self.array

    def Q(self):
        return quaternion.Quaternion(rot2quaternion(self.array))

    def R(self):
        return rotationmatrix.RotationMatrix(self.array[0:3][0:3])

    def pos(self):
        return self.array[0:3, 3]

    # def T(self):
    #     return self.array
