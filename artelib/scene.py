#!/usr/bin/env python
# encoding: utf-8
"""
Base Scene class

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
# import sys
import numpy as np

DELTA_TIME = 50.0/1000.0
import itertools

class Sphere():
    def __init__(self, clientID, handle, pa, pb):
        self.clientID = clientID
        self.handle = handle
        self.position = pb
        self.pa = pa
        self.pb = pb
        la = [True]*150
        lb = [False]*150
        la.extend(lb)
        self.movement_dirs = itertools.cycle(la)

    def next_position(self):
        """
        Move randomly towards from point pa to pb and back
        """
        ps = self.position
        # try to mov towards pa
        if next(self.movement_dirs):
            u = self.pa - ps
        else:
            u = self.pb - ps
        nu = np.linalg.norm(u)
        if nu > 0:
            u = u / nu
        k = 0.01*np.random.rand(1, 1)[0][0]
        self.position = self.position + np.dot(k, u)

    def get_position(self):
        return self.position

    def set_object_position(self, position):
        self.position = position
        errorCode = sim.simxSetObjectPosition(self.clientID, self.handle, -1, position, sim.simx_opmode_oneshot_wait)


