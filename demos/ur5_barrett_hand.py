#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5_barret_hand.ttt scene before running this script.

The script provides an example to move a UR5 robot with a Barret Hand.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from robots.grippers import GripperBarretHand
from robots.simulation import Simulation
from robots.ur5 import RobotUR5

if __name__ == "__main__":
    simulation = Simulation()
    simulation.start()
    robot = RobotUR5(simulation=simulation)
    robot.start()
    gripper = GripperBarretHand(simulation=simulation)
    gripper.start(name='/UR5/Barrett_openCloseJoint')

    target_positions = [[0.4, -0.3, 0.2]]  # drop the piece
    target_orientations = [[np.pi / 2, 0, 0]]

    q0 = np.pi / 8 * np.array([-1, 1, 1, 1, 1, 1])
    # set initial position of robot
    robot.moveAbsJ(q0, precision=True, endpoint=True)

    robot.directkinematics(q=q0)

    # robot.set_target_position_orientation(target_positions[0], target_orientations[0])
    q_path, qd_path = robot.inversekinematics_line(target_position=target_positions[0],
                                          target_orientation=Euler(target_orientations[0]),
                                          q0=q0, vmax=0.5)
    gripper.open(precision=True)
    robot.moveAbsPath(q_path, precision=False, endpoint=False)
    gripper.close(precision=True)
    robot.moveAbsPath(q_path[::-1], precision=True, endpoint=True)
    gripper.open(precision=True)

    simulation.stop()

