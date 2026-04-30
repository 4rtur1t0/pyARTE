#!/usr/bin/env python
# encoding: utf-8
"""
The Euler orientation class
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
# from artelib.tools import euler2rot, euler2q
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


def euler2rot(abg):
    calpha = np.cos(abg[0])
    salpha = np.sin(abg[0])
    cbeta = np.cos(abg[1])
    sbeta = np.sin(abg[1])
    cgamma = np.cos(abg[2])
    sgamma = np.sin(abg[2])
    Rx = np.array([[1, 0, 0], [0, calpha, -salpha], [0, salpha, calpha]])
    Ry = np.array([[cbeta, 0, sbeta], [0, 1, 0], [-sbeta, 0, cbeta]])
    Rz = np.array([[cgamma, -sgamma, 0], [sgamma, cgamma, 0], [0, 0, 1]])
    R = np.matmul(Rx, Ry)
    R = np.matmul(R, Rz)
    return R


def euler2q(abg):
    R = euler2rot(abg=abg)
    Q = rot2quaternion(R)
    return Q