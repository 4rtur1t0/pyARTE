#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

RobotUR5 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021

"""
from robots.robot import Robot
import sim


class RobotDyor(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera):

        Robot.__init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target,
                       max_joint_speeds=None, joint_ranges=None, camera=camera)

    def open_gripper(self, precision=False):
        self.gripper.open_gripper(precision=precision)
        if precision:
            self.wait(10)

    def close_gripper(self, precision=False):
        self.gripper.close_gripper(precision=precision)
        if precision:
            self.wait(10)

    def forward(self):
        wheel_speeds = [10, 10]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)
    def backwards(self):
        wheel_speeds = [-10, -10]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def left(self):
        wheel_speeds = [-10, 10]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def right(self):
        wheel_speeds = [10, -10]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

