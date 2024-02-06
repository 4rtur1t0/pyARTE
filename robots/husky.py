#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/husky_robot.ttt scene before running this script.

@Authors: Víctor Márquez
@Time: February 2024
"""
from robots.robot import Robot


class HuskyRobot(Robot):
    def __init__(self, simulation):
        Robot.__init__(self, simulation=simulation)

    def start(self, base_name='/HUSKY'):
        robotbase = self.simulation.sim.getObject(base_name)
        wheelRL = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointRLW')
        wheelFL = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointFLW')
        wheelRR = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointRRW')
        wheelFR = self.simulation.sim.getObject(base_name + '/' + 'Revolute_jointFRW')
        wheeljoints = []
        wheeljoints.append(wheelRL)
        wheeljoints.append(wheelFL)
        wheeljoints.append(wheelRR)
        wheeljoints.append(wheelFR)
        self.joints = wheeljoints

    def forward(self):
        wheel_speeds = [3, 3 ,3 ,3]
        print('va')
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def backwards(self):
        wheel_speeds = [-3, -3, -3, -3]
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def left(self):
        wheel_speeds = [0, 0, 3, 3]
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])

    def right(self):
        wheel_speeds = [3, 3, 0, 0]
        for i in range(4):
            self.simulation.sim.setJointTargetVelocity(self.joints[i], wheel_speeds[i])