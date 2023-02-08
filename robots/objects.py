#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage different objects in Coppelia simulations

@Authors: Arturo Gil
@Time: April 2023

"""
import sim
import numpy as np


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
        position = np.array(position)
        errorCode = sim.simxSetObjectPosition(self.clientID, self.handle, -1, position, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def set_orientation(self, alpha_beta_gamma):
        errorCode = sim.simxSetObjectOrientation(self.clientID, self.handle, sim.sim_handle_parent, alpha_beta_gamma, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def get_position(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.handle, -1, sim.simx_opmode_oneshot_wait)
        return position

    def get_orientation(self):
        errorCode, orientation = sim.simxGetObjectOrientation(self.clientID, self.handle, sim.sim_handle_parent, sim.simx_opmode_oneshot_wait)
        return orientation

