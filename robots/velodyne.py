#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage a Velodyne LIDAR in Coppelia simulations (a laser sensor)

@Authors: Arturo Gil
@Time: April 2021

"""
import sim


class Velodyne():
    def __init__(self, clientID):
        self.clientID = clientID
        self.handle = None

    def start(self, name='velodyneVPL_16'):
        errorCode, handle = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_oneshot_wait)
        self.handle = handle

    def get_laser_data(self):
        """
        This reads the laserdata signal in Coppelia and returns it.
        The laserdata signal must be defined as in the UR5_velodyne.ttt environment.
        """
        error, data = sim.simxGetStringSignal(self.clientID, 'laserdata', sim.simx_opmode_oneshot_wait)
        # TODO: after unpacking the floats, some more-readable data structure should be built.
        data = sim.simxUnpackFloats(data)
        return data
