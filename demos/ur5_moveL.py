#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

The demo presents a series of target points and uses robot.moveL to follow the targets along a
line in task space.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from robots.grippers import GripperRG2
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def ikineline():
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotUR5(clientID=clientID)
    robot.start()
    gripper = GripperRG2(clientID=clientID)
    gripper.start()
    target_positions = [Vector([0.2, -0.45, 0.4]),
                        Vector([0.6, -0.2, 0.25])]
    target_orientations = [Euler([-np.pi, 0, 0]),
                           Euler([-np.pi/2, 0, -np.pi/2])]
    q0 = np.array([-np.pi/4, -np.pi/8, np.pi/2, 0.1, 0.1, 0.1])
    robot.moveAbsJ(q_target=q0)
    for i in range(len(target_positions)):
        robot.moveL(target_position=target_positions[i], target_orientation=Euler(target_orientations[i]))

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikineline()