#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/abbirb140.ttt scene before running this script.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.grippers import GripperRG2
from robots.simulation import Simulation


def test_check_range1(robot):
    """
    TODO: THE POINTS ARE OUT OF RANGE. THE PLANNING FUNCTION SHOULD TAKE
    THIS INTO ACCOUNT AND PRODUCE AN ERROR IN RED
    """
    positions = [Vector([0.5, -0.5, 1.1]),
                 Vector([-0.5, 0.5, 1.1])]
    orientations = [Euler([0, 0, 0]),
                    Euler([0, 0, 0])]
    # Start with moveJ
    total, partial = robot.check_can_be_reached(target_positions=positions, target_orientations=orientations)
    print('Total: ', total)
    print('Partial: ', partial)


def test_check_range2(robot):
    """
    TODO: THE POINTS ARE OUT OF RANGE. THE PLANNING FUNCTION SHOULD TAKE
    THIS INTO ACCOUNT AND PRODUCE AN ERROR IN RED
    """
    positions = [Vector([0.5, 0.0, 0.7]),
                 Vector([-0.3, 0.5, 1.1])]
    orientations = [Euler([0, np.pi/2, 0]),
                    Euler([0, 0, 0])]
    # Start with moveJ
    total, partial = robot.check_can_be_reached(target_positions=positions, target_orientations=orientations)
    print('Total: ', total)
    print('Partial: ', partial)


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    # Connect to the robot
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    # both targets should not be reachable
    test_check_range1(robot)
    # both targets should be reachable
    test_check_range2(robot)

    simulation.stop()

