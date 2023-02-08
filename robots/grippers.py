#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage the grippers in Coppelia simulations

@Authors: Arturo Gil
@Time: April 2021

"""
import sim


class GripperRG2():
    def __init__(self, clientID):
        self.clientID = clientID
        self.joints = None

    def start(self, name='RG2_openCloseJoint'):
        errorCode, gripper_joint1 = sim.simxGetObjectHandle(self.clientID, name,
                                                            sim.simx_opmode_oneshot_wait)
        self.joints = [gripper_joint1]

    def open(self, precision=False):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetPosition=0.1, operationMode=sim.simx_opmode_oneshot)
        if precision:
            for i in range(10):
                sim.simxSynchronousTrigger(clientID=self.clientID)

    def close(self, precision=False):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetPosition=-0.1, operationMode=sim.simx_opmode_oneshot)
        if precision:
            for i in range(10):
                sim.simxSynchronousTrigger(clientID=self.clientID)


class GripperBarretHand():
    def __init__(self, clientID):
        self.clientID = clientID
        self.joints = None

    def start(self, name='Barrett_openCloseJoint'):
        errorCode, gripper_joint1 = sim.simxGetObjectHandle(self.clientID, name,
                                                            sim.simx_opmode_oneshot_wait)
        errorCode, gripper_joint2 = sim.simxGetObjectHandle(self.clientID, name + '0',
                                                            sim.simx_opmode_oneshot_wait)
        self.joints = [gripper_joint1, gripper_joint2]

    def open(self, precision=False):
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetVelocity=0.5, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[1],
                                       targetVelocity=0.5, operationMode=sim.simx_opmode_oneshot)
        if precision:
            for i in range(10):
                sim.simxSynchronousTrigger(clientID=self.clientID)

    def close(self, precision=False):
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetVelocity=-0.5, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[1],
                                       targetVelocity=-0.5, operationMode=sim.simx_opmode_oneshot)

        if precision:
            for i in range(10):
                sim.simxSynchronousTrigger(clientID=self.clientID)


class SuctionPad():
    """
    A standard suction pad in Coppelia. The suction pad acts on the environment by setting the enabled=true
    variable on the Lua script associated to the Suction pad in Coppelia
    """
    def __init__(self, clientID, joints=None):
        self.clientID = clientID
        self.joints = None

    def start(self):
        return

    def open(self, precision=False):
        """
        Deactivates suction (void): thus suction==0
        The SuctionPad should have a Lua script that reads the integer signal "enable_suction_pad"
        """
        returnCode = sim.simxSetIntegerSignal(self.clientID, 'enable_suction_pad', 0, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def close(self, precision=False):
        """
        activates suction
        """
        returnCode = sim.simxSetIntegerSignal(self.clientID, 'enable_suction_pad', 1, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def suction_on(self):
        """
        Activates suction with an easier to understand name
        """
        returnCode = sim.simxSetIntegerSignal(self.clientID, 'enable_suction_pad', 1, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)

    def suction_off(self):
        """
        DeActivates suction with an easier to understand name
        """
        returnCode = sim.simxSetIntegerSignal(self.clientID, 'enable_suction_pad', 0, sim.simx_opmode_oneshot)
        sim.simxSynchronousTrigger(clientID=self.clientID)


class YouBotGripper():
    def __init__(self, clientID):
        self.clientID = clientID
        self.joints = None

    def start(self, name='youBotGripperJoint'):
        errorCode, gripper_joint1 = sim.simxGetObjectHandle(self.clientID, name+'1',
                                                            sim.simx_opmode_oneshot_wait)
        errorCode, gripper_joint2 = sim.simxGetObjectHandle(self.clientID, name + '2',
                                                            sim.simx_opmode_oneshot_wait)
        self.joints = [gripper_joint1, gripper_joint2]

    def open(self, precision=False):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetPosition=-0.05, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[1],
                                       targetPosition=-0.05, operationMode=sim.simx_opmode_oneshot)
        if precision:
            for i in range(10):
                sim.simxSynchronousTrigger(clientID=self.clientID)

    def close(self, precision=False):
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[0],
                                       targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)
        sim.simxSetJointTargetPosition(clientID=self.clientID, jointHandle=self.joints[1],
                                       targetPosition=0.0, operationMode=sim.simx_opmode_oneshot)

        if precision:
            for i in range(10):
                sim.simxSynchronousTrigger(clientID=self.clientID)

    # def open(self, precision=False):
    #     sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[0],
    #                                    targetVelocity=-0.04, operationMode=sim.simx_opmode_oneshot)
    #     sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[1],
    #                                    targetVelocity=-0.04, operationMode=sim.simx_opmode_oneshot)
    #     if precision:
    #         for i in range(10):
    #             sim.simxSynchronousTrigger(clientID=self.clientID)
    #
    # def close(self, precision=False):
    #     sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[0],
    #                                    targetVelocity=0.04, operationMode=sim.simx_opmode_oneshot)
    #     sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[1],
    #                                    targetVelocity=0.04, operationMode=sim.simx_opmode_oneshot)
    #
    #     if precision:
    #         for i in range(10):
    #             sim.simxSynchronousTrigger(clientID=self.clientID)