#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/mobile_robot.ttt scene before using this class.

@Authors: Arturo Gil
@Time: April 2021
"""
from robots.robot import Robot
import sim


class RobotDyor(Robot):
    def __init__(self, clientID):
        Robot.__init__(self)
        self.clientID = clientID

    def start(self):
        # Get the handles of the relevant objects
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, 'ROBOT_DYOR', sim.simx_opmode_oneshot_wait)
        errorCode, wheel1 = sim.simxGetObjectHandle(self.clientID, 'motor_L', sim.simx_opmode_oneshot_wait)
        errorCode, wheel2 = sim.simxGetObjectHandle(self.clientID, 'motor_R', sim.simx_opmode_oneshot_wait)
        wheeljoints = []
        wheeljoints.append(wheel1)
        wheeljoints.append(wheel2)
        self.joints = wheeljoints

    def forward(self):
        wheel_speeds = [5, 5]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def backwards(self):
        wheel_speeds = [-5, -5]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def left(self):
        wheel_speeds = [-5, 5]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

    def right(self):
        wheel_speeds = [5, -5]
        for i in range(2):
            errorCode = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.joints[i],
                                                       targetVelocity=wheel_speeds[i],
                                                       operationMode=sim.simx_opmode_oneshot)

