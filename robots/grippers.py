#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage the grippers in Coppelia simulations

@Authors: Arturo Gil
@Time: April 2021

"""
import sim
import time

# DELTA_TIME = 50/1000.0


class GripperRG2():
    def __init__(self, clientID, joints):
        self.clientID = clientID
        self.joints = joints

    def open_gripper(self, precision=False):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetPosition=0.04, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointMaxForce(clientID=self.clientID, jointHandle=self.joints[0],
                                 force=50.0, operationMode=sim.simx_opmode_oneshot)
        if precision:
            self.wait(10)

    def close_gripper(self, precision=False):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetPosition=-0.04, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointMaxForce(clientID=self.clientID, jointHandle=self.joints[0],
                                 force=50.0, operationMode=sim.simx_opmode_oneshot)
        if precision:
            self.wait(10)

    def wait(self, steps=1):
        for i in range(steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)


class GripperBarretHand():
    def __init__(self, clientID, joints):
        self.clientID = clientID
        self.joints = joints

    def open_gripper(self, precision=False):
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetVelocity=0.5, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[1],
                                       targetVelocity=0.5, operationMode=sim.simx_opmode_oneshot)
        if precision:
            self.wait(15)

    def close_gripper(self, precision=False):
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetVelocity=-0.5, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[1],
                                       targetVelocity=-0.5, operationMode=sim.simx_opmode_oneshot)

        if precision:
            self.wait(15)

    def wait(self, steps=1):
        for i in range(steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)



class SuctionPad():
    """
    A standard suction pad in Coppelia. The suction pad acts on the environment by setting the enabled=true
    variable on the Lua script associated to the Suction pad in Coppelia
    """
    def __init__(self, clientID, joints=None):
        self.clientID = clientID
        self.joints = joints

    def open_gripper(self, precision=False):
        """
        Deactivates suction (void): thus suction==0
        """
        returnCode = sim.simxSetIntegerSignal(self.clientID, 'enable_suction_pad', 0, sim.simx_opmode_oneshot)

    def close_gripper(self, precision=False):
        """
        activates suction
        """
        returnCode = sim.simxSetIntegerSignal(self.clientID, 'enable_suction_pad', 1, sim.simx_opmode_oneshot)

    def wait(self, steps=1):
        for i in range(steps):
            sim.simxSynchronousTrigger(clientID=self.clientID)