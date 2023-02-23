#!/usr/bin/env python
# encoding: utf-8
"""
Stablish a connection with Coppelia.

@Authors: Arturo Gil
@Time: April 2021
"""
import time
import sim
import sys


class Simulation():
    def __init__(self):
        self.clientID = None

    def start(self):
        """
        Connect python to the server running on Coppelia.
        """
        # Python connect to the V-REP client
        sim.simxFinish(-1)
        clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)
        self.clientID = clientID
        if clientID != -1:
            print("Connected to remote API server")
            # stop previous simiulation
            sim.simxStopSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
            time.sleep(3)
            sim.simxStartSimulation(clientID=clientID, operationMode=sim.simx_opmode_blocking)
            # enable the synchronous mode
            sim.simxSynchronous(clientID=clientID, enable=True)
        else:
            print("Connection not successful")
            sys.exit("Connection failed,program ended!")
        return clientID

    def stop(self):
        sim.simxStopSimulation(self.clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(self.clientID)

    def wait(self, steps=1):
        """
        Wait n simulation steps.
        """
        for i in range(0, steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)

