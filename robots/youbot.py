#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/youbot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
import time
import sim
import sys
import numpy as np
from robots.robot import Robot
# standard delta time for Coppelia, please modify if necessary
DELTA_TIME = 50.0/1000.0


class YouBotRobot(Robot):
    def __init__(self, clientID):
        Robot.__init__(self)
        self.clientID = clientID
        self.wheeljoints = None
        self.joints = None
        self.base = None
        self.gripper = None
        self.joint_directions = None
        # complete all data from base class
        self.epsilonq = 0.005

    def start(self, base_name='/youBot', joint_name='youBotArmJoint'):
        armjoints = []
        # Get the handles of the relevant objects
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, base_name, sim.simx_opmode_oneshot_wait)
        # handles to the wheels
        errorCode, fl = sim.simxGetObjectHandle(self.clientID, base_name + '/rollingJoint_fl',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, rl = sim.simxGetObjectHandle(self.clientID, base_name + '/rollingJoint_rl',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, rr = sim.simxGetObjectHandle(self.clientID, base_name + '/rollingJoint_rr',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, fr = sim.simxGetObjectHandle(self.clientID, base_name + '/rollingJoint_fr',
                                                sim.simx_opmode_oneshot_wait)
        wheeljoints = [fl, rl, rr, fr]

        errorCode, q1 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '0',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q2 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '1',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q3 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '2',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q4 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '3',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q5 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '4',
                                                sim.simx_opmode_oneshot_wait)
        # handles to the armjoints
        armjoints.append(q1)
        armjoints.append(q2)
        armjoints.append(q3)
        armjoints.append(q4)
        armjoints.append(q5)

        # must store the joints
        self.wheeljoints = wheeljoints
        self.joints = armjoints
        self.base = robotbase
        # self.gripper = gripper
        self.joint_directions = [1, -1, -1, -1, 1]


    def set_base_speed(self, forwardspeed, lefrigthspeed, rotspeed):
        """
        Given a speed in forward/backward direction
        - left/right directino
        and - rotation speed
        Computes the speeds of each wheel so a to apply the commanded speed to the robot base.
        """

        error = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[0],
                                               targetVelocity=-forwardspeed - lefrigthspeed - rotspeed,
                                               operationMode=sim.simx_opmode_oneshot)
        error = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[1],
                                               targetVelocity=-forwardspeed + lefrigthspeed - rotspeed,
                                               operationMode=sim.simx_opmode_oneshot)
        error = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[2],
                                               targetVelocity=-forwardspeed - lefrigthspeed + rotspeed,
                                               operationMode=sim.simx_opmode_oneshot)
        error = sim.simxSetJointTargetVelocity(clientID=self.clientID, jointHandle=self.wheeljoints[3],
                                               targetVelocity=-forwardspeed + lefrigthspeed + rotspeed,
                                               operationMode=sim.simx_opmode_oneshot)




