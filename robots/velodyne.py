#!/usr/bin/env python
# encoding: utf-8
"""
Classes to manage a Velodyne LIDAR in Coppelia simulations (a laser sensor)

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np


class Velodyne():
    def __init__(self, simulation):
        self.simulation = simulation
        self.handle = None

    def start(self, name='velodyneVPL_16'):
        handle = self.simulation.sim.getObject(name)
        self.handle = handle

    def get_laser_data(self):
        """
        This reads the laserdata signal in Coppelia and returns it.
        The laserdata signal must be defined as in the UR5_velodyne.ttt environment.
        """
        for i in range(5):
            data = self.simulation.sim.readCustomDataBlock(self.simulation.sim.handle_scene, "laserdata")
            data = self.simulation.sim.unpackFloatTable(data)
            if data != None:
                # reshape to 3D points
                data = np.reshape(data, (-1, 3))
                return data

