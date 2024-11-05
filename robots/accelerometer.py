#!/usr/bin/env python
# encoding: utf-8
"""
Classes to manage an accelerometer in Coppelia simulations (a LiDAR sensor)

@Authors: Arturo Gil
@Time: April 2024
"""
import numpy as np
import open3d as o3d


class Accelerometer():
    def __init__(self, simulation):
        self.simulation = simulation
        self.handle = None

    def start(self, name='/Accelerometer'):
        handle = self.simulation.sim.getObject(name)
        self.handle = handle

    def get_accel_data(self):
        """
        This reads the acceldata signal in Coppelia and returns it.
        The acceldata signal must be defined in the accelerometer
        """
        data = self.simulation.sim.readCustomDataBlock(self.simulation.sim.handle_scene, "acceldata")
        data = self.simulation.sim.unpackFloatTable(data)
        if data != None:
            # reshape to 3D points
            data = np.reshape(data, (-1, 3))
            # self.pointcloud.points = o3d.utility.Vector3dVector(data)
            return data
        else:
            return None




