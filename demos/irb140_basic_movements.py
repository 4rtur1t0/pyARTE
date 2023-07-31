#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/irb140.ttt scene before running this script.

@Authors: Arturo Gil arturo.gil@umh.es
@Time: April 2021
"""
import numpy as np
from robots.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2
from robots.simulation import Simulation


def pick_and_place_rep():
    """
    A repeated pick and place application.
    """
    simulation = Simulation()
    simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start()

    q0 = np.array([0.0, 0.2, np.pi / 4, 0.1, 0.1, np.pi/2])
    robot.moveAbsJ(q_target=q0)
    gripper.open(precision=True)

    q0 = np.array([0.0, 0.5, 0.25, 0.1, 0.8, np.pi / 2])
    robot.moveAbsJ(q_target=q0, precision=True)
    gripper.close(precision=True)
    q0 = np.array([0.0, 0.2, 0, 0, 0, np.pi / 2])
    robot.moveAbsJ(q_target=q0, precision=True)
    gripper.open(precision=True)

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    pick_and_place_rep()
