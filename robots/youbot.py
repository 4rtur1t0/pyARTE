#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/youbot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2021

"""
from robots.robot import Robot
import numpy as np


class YouBotBase(Robot):
    def __init__(self, simulation):
        Robot.__init__(self, simulation)
        self.simulation = simulation
        self.wheeljoints = None
        self.base = None
        self.dummy = None
        self.gripper = None
        self.joint_directions = None
        # complete all data from base class
        self.epsilonq = 0.005
        self.r = 0.045 # mecanum wheel radius/swedish
        # self.b =

    def start(self, base_name='/youBot', joint_name='youBotArmJoint'):
        armjoints = []
        # Get the handles of the relevant objects
        robotbase = self.simulation.sim.getObject(base_name)
        youbotref = self.simulation.sim.getObject('/youBot/youBot_ref')
        # handles to the wheels
        fl = self.simulation.sim.getObject(base_name + '/rollingJoint_fl')
        rl = self.simulation.sim.getObject(base_name + '/rollingJoint_rl')
        rr = self.simulation.sim.getObject(base_name + '/rollingJoint_rr')
        fr = self.simulation.sim.getObject(base_name + '/rollingJoint_fr')
        wheeljoints = [fl, rl, rr, fr]

        # must store the joints
        self.wheeljoints = wheeljoints
        self.base = robotbase
        self.dummy = youbotref
        self.joint_directions = [1, -1, -1, -1, 1]

    def set_base_speed(self, forwardspeed, leftrigthspeed, rotspeed):
        """
        Given a speed in forward/backward direction
        - left/right directino
        and - rotation speed
        Computes the speeds of each wheel so a to apply the commanded speed to the robot base.
        """
        # B = np.array([b+d/r])
        # V = np.array([forwardspeed, leftrigthspeed, rotspeed])
        # w = np.dot(B, V)
        self.simulation.sim.setJointTargetVelocity(self.wheeljoints[0], forwardspeed - leftrigthspeed - rotspeed)
        self.simulation.sim.setJointTargetVelocity(self.wheeljoints[1], -forwardspeed + leftrigthspeed - rotspeed)
        self.simulation.sim.setJointTargetVelocity(self.wheeljoints[2], -forwardspeed - leftrigthspeed + rotspeed)
        self.simulation.sim.setJointTargetVelocity(self.wheeljoints[3], -forwardspeed + leftrigthspeed + rotspeed)

    def get_true_position_and_orientation(self):
        error, position = self.simulation.sim.getObjectPosition(objectHandle=self.dummy, relativeToObjectHandle=-1)
        error, orientation = self.simulation.sim.simxGetObjectOrientation(objectHandle=self.dummy,
                                             relativeToObjectHandle=-1)
        return position, orientation


class YouBotArm(Robot):
    def __init__(self, simulation):
        Robot.__init__(self, simulation)
        self.simulation = simulation
        self.joints = None
        self.base = None
        self.joint_directions = None
        # complete all data from base class
        self.epsilonq = 0.005

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([100, 100, 100, 100, 100, 100])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges. joints 4 and 6 can be configured as unlimited
        # default joint limits:
        # q1 (+-180), q2 (-90,110), q3 (-230, 50), q4 (+-200), q5 (+-115), q6 (+-400)
        # here, the joint range for q4 has been extended
        joint_ranges = np.array([[-180, -90, -230, -400, -115, -400],
                                 [180,   110, 110,  400,  115, 400]])
        self.joint_ranges = joint_ranges * np.pi / 180.0


    def start(self, base_name='/youBot', joint_name='youBotArmJoint'):
        armjoints = []
        # Get the handles of the relevant objects
        robotbase = self.simulation.sim.getObject(base_name)

        q1 = self.simulation.sim.getObject(base_name + '/' + joint_name + '0')
        q2 = self.simulation.sim.getObject(base_name + '/' + joint_name + '1')
        q3 = self.simulation.sim.getObject(base_name + '/' + joint_name + '2')
        q4 = self.simulation.sim.getObject(base_name + '/' + joint_name + '3')
        q5 = self.simulation.sim.getObject(base_name + '/' + joint_name + '4')
        # handles to the armjoints
        armjoints.append(q1)
        armjoints.append(q2)
        armjoints.append(q3)
        armjoints.append(q4)
        armjoints.append(q5)

        # must store the joints
        self.joints = armjoints
        self.base = robotbase
        self.joint_directions = [1, -1, -1, -1, 1]

