#!/usr/bin/env python
# encoding: utf-8
"""
A DH Serial Robot
A DH SerialLink
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.tools import rot2quaternion
from artelib import euler, quaternion, homogeneousmatrix


class SerialRobot():
    def __init__(self, n, T0, name):
        self.name = name
        self.n = n
        self.T0 = HomogeneousMatrix(T0)
        # self.TCP = HomogeneousMatrix(TCP)
        self.transformations = []

    def __str__(self):
        return str(self.name), 'robot with ', str(self.n), ' DOF'

    def append(self, th, d, a, alpha, link_type='R'):
        sl = SerialLink(th, d, a, alpha, link_type=link_type)
        self.transformations.append(sl)

    def directkinematics(self, q):
        T = self.T0
        for i in range(len(self.transformations)):
            A = self.transformations[i].dh(q[i])
            T = T*A
            # print('A', A)
            # print('T', T)
        # print(T)
        return T

    def dh(self, q, i):
        return self.transformations[i].dh(q[i])

    def get_link_type(self, i):
        return self.transformations[i].link_type


class SerialLink():
    def __init__(self, th, d, a, alpha, link_type='R'):
        self.th = th
        self.d = d
        self.a = a
        self.alpha = alpha
        self.link_type = link_type

    def __str__(self):
        return print('Seriallink with: '), str(self.th) , self.d, self.a, self.alpha

    def dh(self, q):
        """
        Here, q is the ith variable in the joint position vector used to compute the ith DH transformation matrix.
        """
        # if rotational
        if self.link_type == 'R':
            theta = q + self.th
            d = self.d
        # translational
        elif self.link_type == 'P':
            theta = self.th
            d = q + self.d
        else:
            print('Unknown link type')
            theta = self.th
            d = self.d

        A = np.array([[np.cos(theta), -np.cos(self.alpha)*np.sin(theta),   np.sin(self.alpha)*np.sin(theta),   self.a*np.cos(theta)],
                      [np.sin(theta),  np.cos(self.alpha)*np.cos(theta), -np.sin(self.alpha)*np.cos(theta), self.a*np.sin(theta)],
                      [0,                    np.sin(self.alpha),                      np.cos(self.alpha),          d],
                      [0,        0,        0,        1]])
        return homogeneousmatrix.HomogeneousMatrix(A)



