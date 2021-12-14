#!/usr/bin/env python
# encoding: utf-8
"""
Configuration function for scenes/ur5.ttt.

@Authors: Arturo Gil
@Time: April 2021

"""
import time
import sim
import sys

from artelib.kukalbr import RobotKUKALBR
from artelib.ur5 import RobotUR5
from artelib.scene import Scene

# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


def init_simulation_UR5():
    # Python connect to the V-REP client
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

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

    armjoints = []
    gripper = []
    objects = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'UR5', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'UR5_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'UR5_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'UR5_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'UR5_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'UR5_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'UR5_joint6', sim.simx_opmode_oneshot_wait)

    errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)

    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)

    # errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)

    gripper.append(gripper1)

    # objects.append(sphere)
    robot = RobotUR5(clientID=clientID, wheeljoints=[],
                    armjoints=armjoints, base=robotbase,
                    end_effector=end_effector, gripper=gripper, target=target, camera=camera)
    scene = Scene(clientID=clientID, objects=objects)
    return robot, scene



def init_simulation_KUKALBR():
    # Python connect to the V-REP client
    sim.simxFinish(-1)
    clientID = sim.simxStart('127.0.0.1', 19997, True, True, 5000, 5)

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

    armjoints = []
    gripper = []
    # Get the handles of the relevant objects
    errorCode, robotbase = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820', sim.simx_opmode_oneshot_wait)
    errorCode, end_effector = sim.simxGetObjectHandle(clientID, 'end_effector', sim.simx_opmode_oneshot_wait)

    errorCode, q1 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint1', sim.simx_opmode_oneshot_wait)
    errorCode, q2 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint2', sim.simx_opmode_oneshot_wait)
    errorCode, q3 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint3', sim.simx_opmode_oneshot_wait)
    errorCode, q4 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint4', sim.simx_opmode_oneshot_wait)
    errorCode, q5 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint5', sim.simx_opmode_oneshot_wait)
    errorCode, q6 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint6', sim.simx_opmode_oneshot_wait)
    errorCode, q7 = sim.simxGetObjectHandle(clientID, 'LBR_iiwa_14_R820_joint7', sim.simx_opmode_oneshot_wait)
    errorCode, gripper1 = sim.simxGetObjectHandle(clientID, 'RG2_openCloseJoint', sim.simx_opmode_oneshot_wait)
    errorCode, target = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_oneshot_wait)
    errorCode, camera = sim.simxGetObjectHandle(clientID, 'camera', sim.simx_opmode_oneshot_wait)
    errorCode, sphere = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_oneshot_wait)

    armjoints.append(q1)
    armjoints.append(q2)
    armjoints.append(q3)
    armjoints.append(q4)
    armjoints.append(q5)
    armjoints.append(q6)
    armjoints.append(q7)
    gripper.append(gripper1)

    robot = RobotKUKALBR(clientID=clientID, wheeljoints=[],
                         armjoints=armjoints, base=robotbase,
                         end_effector=end_effector, gripper=gripper,
                         target=target, camera=camera)

    objects = []
    objects.append(sphere)
    scene = Scene(clientID=clientID, objects=objects)
    return robot, scene
