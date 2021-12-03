#!/usr/bin/env python
# encoding: utf-8
"""
Base Scene class

@Authors: Arturo Gil
@Time: April 2021

"""
# import time
import sim
# import sys
import numpy as np
# standard delta time for Coppelia, please modify if necessary
# from artelib.artelib import compute_w_between_orientations, euler2Rot
# from kinematics.jacobians_ur5 import eval_symbolic_jacobian_UR5
# import matplotlib.pyplot as plt


DELTA_TIME = 50.0/1000.0


class Scene():
    def __init__(self, clientID, objects):
        self.clientID = clientID
        self.objects = objects
        self.angle = 2.5

    def random_walk(self):
        errorCode, position = sim.simxGetObjectPosition(self.clientID, self.objects[0], -1, sim.simx_opmode_oneshot_wait)
        v = np.array([np.cos(self.angle), np.sin(self.angle), 0])
        # position
        position = np.array(position) + 0.1*v
        self.angle = self.angle + 0.1*np.random.rand(1, 1)
        errorCode = sim.simxSetObjectPosition(self.clientID, self.objects[0], -1, position, sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def stop_simulation(self):
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(self.clientID)

