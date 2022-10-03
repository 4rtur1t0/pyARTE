"""
Dummy movement of a Reference system using Euler angles XYZ in local coordinates
Use with scenes/euler.ttt
"""
from sceneconfig.scene_configs import init_sim
import sim
import numpy as np


def move_reference_frame():
    clientID = init_sim()
    position = [0, 0, 0.2]

    # Get the handles of the relevant objects
    errorCode, ref_frame = sim.simxGetObjectHandle(clientID, 'ReferenceFrame', sim.simx_opmode_oneshot_wait)

    errorCode = sim.simxSetObjectPosition(clientID=clientID, objectHandle=ref_frame,
                                          relativeToObjectHandle=-1, position=position,
                                          operationMode=sim.simx_opmode_oneshot_wait)
    alpha = 0
    beta = 0
    gamma = 0

    for i in range(100):
        alpha += .005
        errorCode = sim.simxSetObjectOrientation(clientID=clientID, objectHandle=ref_frame,
                                                 eulerAngles=[alpha, beta, gamma], relativeToObjectHandle=-1,
                                                 operationMode=sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=clientID)

    for i in range(100):
        beta += .005
        errorCode = sim.simxSetObjectOrientation(clientID=clientID, objectHandle=ref_frame,
                                                 eulerAngles=[alpha, beta, gamma], relativeToObjectHandle=-1,
                                                 operationMode=sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=clientID)
    for i in range(100):
        gamma += .005
        errorCode = sim.simxSetObjectOrientation(clientID=clientID, objectHandle=ref_frame,
                                                 eulerAngles=[alpha, beta, gamma], relativeToObjectHandle=-1,
                                                 operationMode=sim.simx_opmode_oneshot_wait)
        sim.simxSynchronousTrigger(clientID=clientID)

        sim.simxStopSimulation(clientID, sim.simx_opmode_oneshot_wait)
        sim.simxFinish(clientID)

if __name__ == "__main__":
    move_reference_frame()