#!/usr/bin/env python
# encoding: utf-8
"""
The Euler orientation class
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.tools import euler2rot, euler2q
from artelib import quaternion, rotationmatrix


class Euler():
    def __init__(self, abg):
        if isinstance(abg, list):
            self.abg = np.array(abg)
        elif isinstance(abg, np.ndarray):
            self.abg = abg
        elif isinstance(abg, Euler):
            self.abg = abg.abg

    def R(self):
        return rotationmatrix.RotationMatrix(euler2rot(self.abg))

    def Q(self):
        return quaternion.Quaternion(euler2q(self.abg))

    def __str__(self):
        return str(self.abg)
