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
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def test_moveL_z0(robot):
    """
    Test moveL on lines going with the wrist through Z0
    """
    positions = [
                 Vector([0.3, 0.3, 1.2]), # 0
                 Vector([-0.2, -0.2, 1.2]),
                 Vector([0.3, -0.3, 1.0]),# 2
                 Vector([-0.3, 0.3, 1.0]),
                 Vector([0.4, 0, 1.0]), # 4
                 Vector([-0.4, 0, 1.0]),
                 Vector([0.4, -0.065, 0.8]), # 6
                 Vector([-0.4, -0.065, 0.8]),
                 Vector([0.4, -0.065, 0.8]), # 8
                 Vector([-0.4, -0.065, 0.8]),
                 Vector([0.6, -0.2, 0.8]), # 10
                 Vector([0.6, 0.2, 0.8])
                 ]
    orientations = [
                    Euler([0, 0, 0]), # 0
                    Euler([0, 0, 0]),
                    Euler([0, 0, 0]), # 2
                    Euler([0, 0, 0]),
                    Euler([0, 0, 0]), # 4
                    Euler([0, 0, 0]),
                    Euler([np.pi / 2, 0, 0]), # 6
                    Euler([np.pi / 2, 0, 0]),
                    Euler([np.pi / 2, 0, 0]), # 8
                    Euler([np.pi / 2, 0, 0]),
                    Euler([0, 0, 0]), # 10
                    Euler([0, 0, 0])
                    ]
    # Start with moveJ
    for i in range(0, len(positions), 2):
        print('TARGETING POINTS: ', i, i+1)
        robot.moveJ(target_position=positions[i], target_orientation=orientations[i],
                    precision=True)
        robot.moveL(target_position=positions[i+1], target_orientation=orientations[i+1],
                    precision=False,
                    plot=False,
                    speed_factor=3.0,
                    debug=False)


if __name__ == "__main__":
    # Start simulation
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    # Connect to the robot
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    test_moveL_z0(robot)

    simulation.stop()

