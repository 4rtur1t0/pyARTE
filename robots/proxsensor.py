import sim


class ProxSensor():
    def __init__(self, clientID):
        self.clientID = clientID
        self.proxsensor = None

    def start(self, name='prox_sensor'):
        errorCode, prox_sensor = sim.simxGetObjectHandle(self.clientID, name, sim.simx_opmode_oneshot_wait)
        self.proxsensor = prox_sensor

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
        return point