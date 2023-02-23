#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage different objects in Coppelia simulations

@Authors: Arturo Gil
@Time: April 2023

"""
import sim
import numpy as np
from artelib import rotationmatrix
from artelib import homogeneousmatrix
from artelib import vector
from artelib import euler


class Sphere():
    def __init__(self, clientID):
        self.clientID = clientID
        self.camera = None

    def start(self, name='/Sphere'):
        errorCode, handle = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_oneshot_wait)
        self.handle = handle

    def set_position(self, position):
        position = np.array(position)
        errorCode = sim.simxSetObjectPosition(self.clientID, self.handle, -1, position, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_orientation(self, alpha_beta_gamma):
        errorCode = sim.simxSetObjectOrientation(self.clientID, self.handle, sim.sim_handle_parent, alpha_beta_gamma, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def get_position(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)
        return position


class ReferenceFrame():
    def __init__(self, clientID):
        self.clientID = clientID
        self.handle = None

    def start(self):
        # Get the handles of the relevant objects
        errorCode, handle = sim.simxGetObjectHandle(self.clientID, 'ReferenceFrame', sim.simx_opmode_oneshot_wait)
        self.handle = handle

    def set_position(self, position):
        if isinstance(position, np.ndarray):
            position = np.array(position)
        elif isinstance(position, list):
            position = np.array(position)
        elif isinstance(position, vector.Vector):
            position = position.array
        elif isinstance(position, homogeneousmatrix.HomogeneousMatrix):
            position = position.pos
        errorCode = sim.simxSetObjectPosition(self.clientID, self.handle, -1, position, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_orientation(self, orientation):
        # if orientation is given by a RotationMatrix
        if isinstance(orientation, rotationmatrix.RotationMatrix):
            abg = orientation.euler()[0]
            abg = abg.abg
        elif isinstance(orientation, list):
            abg = np.array(orientation)
        elif isinstance(orientation, euler.Euler):
            abg = orientation.abg
        errorCode = sim.simxSetObjectOrientation(self.clientID, self.handle, sim.sim_handle_parent, abg, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_position_and_orientation(self, *args):
        if len(args) == 1:
            if isinstance(args[0], homogeneousmatrix.HomogeneousMatrix):
                position = args[0].pos()
                orientation = args[0].R().euler()[0].abg
        elif len(args) == 2:
            position = args[0]
            orientation = args[1]
            if isinstance(position, list):
                position = np.array(position)
            elif isinstance(position, vector.Vector):
                position = np.array(position.array)
            if isinstance(orientation, euler.Euler):
                orientation = orientation.abg
            elif isinstance(orientation, rotationmatrix.RotationMatrix):
                orientation = orientation.euler()[0].abg
            else:
                raise Exception

        errorCode = sim.simxSetObjectPosition(self.clientID, self.handle, -1, position,
                                                      sim.simx_opmode_oneshot_wait)
        errorCode = sim.simxSetObjectOrientation(self.clientID, self.handle, sim.sim_handle_parent, orientation,
                                                     sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def get_position(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)
        return position

    def get_orientation(self):
        errorCode, orientation = sim.simxGetObjectOrientation(self.clientID, self.handle, sim.sim_handle_parent, sim.simx_opmode_oneshot_wait)
        return orientation

