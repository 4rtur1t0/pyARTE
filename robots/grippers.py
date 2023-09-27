#!/usr/bin/env python
# encoding: utf-8
"""

Classes to manage the grippers in Coppelia simulations

@Authors: Arturo Gil
@Time: April 2021

"""


class GripperRG2():
    def __init__(self, simulation):
        self.simulation = simulation
        self.joints = None

    def start(self, name='/IRB140/RG2_openCloseJoint'):
        gripper_joint1 = self.simulation.sim.getObject(name)
        self.joints = [gripper_joint1]

    def open(self, precision=True):
        self.simulation.sim.setJointTargetVelocity(self.joints[0], 0.1)
        if precision:
            self.simulation.wait(steps=20)

    def close(self, precision=False):
        self.simulation.sim.setJointTargetVelocity(self.joints[0], -0.1)
        if precision:
            self.simulation.wait(steps=20)


class GripperBarretHand():
    def __init__(self, simulation):
        self.simulation = simulation
        self.joints = None

    def start(self, name='Barrett_openCloseJoint'):
        gripper_joint1 = self.simulation.sim.getObject(name)
        gripper_joint2 = self.simulation.sim.getObject(name + '0')
        self.joints = [gripper_joint1, gripper_joint2]

    def open(self, precision=False):
        self.simulation.sim.setJointTargetVelocity(self.joints[0], 0.1)
        self.simulation.sim.setJointTargetVelocity(self.joints[1], 0.1)
        if precision:
            self.simulation.wait(20)

    def close(self, precision=False):
        self.simulation.sim.setJointTargetVelocity(self.joints[0], -0.1)
        self.simulation.sim.setJointTargetVelocity(self.joints[1], -0.1)
        if precision:
            self.simulation.wait(20)


class SuctionPad():
    """
    A standard suction pad in Coppelia. The suction pad acts on the environment by setting the enabled=true
    variable on the Lua script associated to the Suction pad in Coppelia
    """
    def __init__(self, simulation, joints=None):
        self.simulation = simulation
        self.joints = None

    def start(self):
        return

    def open(self, precision=False):
        """
        Deactivates suction (void): thus suction==0
        The SuctionPad should have a Lua script that reads the integer signal "enable_suction_pad"
        """
        returnCode = self.simulation.sim.setIntegerSignal('enable_suction_pad', 0)
        # sim.simxSynchronousTrigger(clientID=self.clientID)

    def close(self, precision=False):
        """
        activates suction
        """
        returnCode = self.simulation.sim.setIntegerSignal('enable_suction_pad', 1)
        # sim.simxSynchronousTrigger(clientID=self.clientID)

    def suction_on(self):
        """
        Activates suction with an easier to understand name
        """
        returnCode = self.simulation.sim.setIntegerSignal('enable_suction_pad', 1)
        # sim.simxSynchronousTrigger(clientID=self.clientID)

    def suction_off(self):
        """
        DeActivates suction with an easier to understand name
        """
        returnCode = self.simulation.sim.setIntegerSignal('enable_suction_pad', 0)
        # sim.simxSynchronousTrigger(clientID=self.clientID)


class YouBotGripper():
    def __init__(self, simulation):
        self.simulation = simulation
        self.joints = None

    def start(self, name='/youBot/youBotGripperJoint'):
        gripper_joint1 = self.simulation.sim.getObject(name+'1')
        gripper_joint2 = self.simulation.sim.getObject(name + '2')
        self.joints = [gripper_joint1, gripper_joint2]

    def open(self, precision=False):
        self.simulation.sim.setJointTargetPosition(self.joints[0], -0.05)
        self.simulation.sim.setJointTargetPosition(self.joints[1], -0.05)
        if precision:
            self.simulation.wait(10)

    def close(self, precision=False):
        self.simulation.sim.setJointTargetPosition(self.joints[0], 0.0)
        self.simulation.sim.setJointTargetPosition(self.joints[1], 0.0)
        if precision:
            self.simulation.wait(10)

