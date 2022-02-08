#!/usr/bin/env python
# encoding: utf-8
"""
The orientation class
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.tools import euler2rot, euler2q, quaternion2rot, q2euler, rot2quaternion


class Euler():
    def __init__(self, abg):
        self.abg = np.array(abg)

    def R(self):
        return euler2rot(self.abg)

    def Q(self):
        return euler2q(self.abg)


class Quaternion():
    def __init__(self, quaternion):
        self.quaternion = np.array(quaternion)

    def R(self):
        return quaternion2rot(self.quaternion)

    def Euler(self):
        return Euler(q2euler(self.quaternion))


class RotationMatrix():
    def __init__(self, rot):
        self.rot = np.array(rot[0:3, 0:3])

    def R(self):
        return self.rot

    def Q(self):
        return rot2quaternion(self.rot)