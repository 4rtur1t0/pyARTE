
import sim
class ProxSensor():
    def __init__(self, clientID, proxsensor):
        self.clientID = clientID
        self.proxsensor = proxsensor



    def is_activated(self):
        code, state, point, handle, snv = sim.simxReadProximitySensor(clientID=self.clientID,
                                                                      sensorHandle=self.proxsensor,
                                                                      operationMode=sim.simx_opmode_oneshot_wait)
        return state

    def readstate(self):
        code, state, point, handle, snv = sim.simxReadProximitySensor(clientID=self.clientID,
                                                                      sensorHandle=self.proxsensor,
                                                                      operationMode=sim.simx_opmode_oneshot_wait)
        return state

    def readvalues(self):
        code, state, point, handle, snv = sim.simxReadProximitySensor(clientID=self.clientID,
                                                                      sensorHandle=self.proxsensor,
                                                                      operationMode=sim.simx_opmode_oneshot_wait)
        print(code)
        print(state)
        print(point)
        return point