#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/mobile_robot.ttt scene before using this class.

@Authors: Arturo Gil
@Time: April 2021
"""
from robots.robot import Robot


class RobotDyor(Robot):
    def __init__(self, simulation):
        Robot.__init__(self, simulation=simulation)

    def start(self, base_name='/ROBOT_DYOR'):
        robotbase = self.simulation.sim.getObject(base_name)
        wheel1 = self.simulation.sim.getObject(base_name + '/' + 'motor_L')
        wheel2 = self.simulation.sim.getObject(base_name + '/' + 'motor_R')
        wheeljoints = []
        wheeljoints.append(wheel1)
        wheeljoints.append(wheel2)
        self.joints = wheeljoints

    def forward(self):
        wheel_speeds = [5, 5]
        for i in range(2):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def backwards(self):
        wheel_speeds = [-5, -5]
        for i in range(2):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def left(self):
        wheel_speeds = [-5, 5]
        for i in range(2):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def right(self):
        wheel_speeds = [5, -5]
        for i in range(2):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

