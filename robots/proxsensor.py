"""
A class to wrap a proximity sensor in Coppelia

Author: Arturo Gil
Date: 2022

Rev: July 2023, modified to ZMQ api
"""


class ProxSensor():
    def __init__(self, simulation):
        self.simulation = simulation
        self.proxsensor = None

    def start(self, name='/conveyor/prox_sensor'):
        prox_sensor = self.simulation.sim.getObject(name)
        self.proxsensor = prox_sensor

    def is_activated(self):
        # code, state, point, handle, snv = self.simulation.sim.readProximitySensor(self.proxsensor)
        result = self.simulation.sim.readProximitySensor(self.proxsensor)
        return result[0]

    def readstate(self):
        code, state, point, handle, snv = self.simulation.sim.readProximitySensor(self.proxsensor)
        return state

    def readvalues(self):
        code, state, point, handle, snv = self.simulation.sim.simxReadProximitySensor(self.proxsensor)
        return point