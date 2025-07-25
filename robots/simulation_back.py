#!/usr/bin/env python
# encoding: utf-8
"""
Stablish a connection with Coppelia.

@Authors: Arturo Gil
@Time: April 2021
"""
import time
from coppeliasim_zmqremoteapi_client import *


class Simulation():
    def __init__(self):
        self.sim = None
        self.client = None
        self.simulation_speed = 3

    def start(self):
        """
        Connect python to the server running on Coppelia.
        """
        # Python connect to the V-REP client and start simulation
        self.client = RemoteAPIClient()
        self.sim = self.client.getObject('sim')
        # self.client.setStepping(True)
        state = self.sim.getSimulationState()

        # simulation is stopped
        if state == 0:
            self.sim.startSimulation()
        else:
            # try to stop the simulation if is in a zombie state
            self.sim.stopSimulation()
            time.sleep(3)
            self.sim.startSimulation()
        # apply stepping True after the simulation is actually created
        self.client.setStepping(True)
        # Modify simulation speed to get a nicer result
        # self.sim.setInt32Param(self.sim.intparam_speedmodifier, self.simulation_speed)
        print('CONNECTED TO COPPELIA!')

    def stop(self):
        print('STOPPING SIMULATION!')
        self.sim.stopSimulation()

    def wait(self, steps=1):
        """
        Wait n simulation steps.
        """
        for i in range(0, steps):
            self.client.step()

    def wait_time(self, seconds=1):
        """
        Wait in seconds
        """
        t1 = self.sim.getSimulationTime()
        while True:
            t2 = self.sim.getSimulationTime()
            if (t2-t1) >= seconds:
                break
            self.client.step()

    def get_simulation_time_step(self):
        return self.sim.getSimulationTimeStep()
