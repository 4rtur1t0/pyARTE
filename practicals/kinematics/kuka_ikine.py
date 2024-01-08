#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_lbr_iiwa_R800.ttt scene before running this script.

This demo presents a simple inverse kinematic scheme with the KUKA IIWA LBR arm

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def ikine():
    simulation = Simulation()
    simulation.start()
    robot = RobotKUKALBR(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start(name='/LBR_iiwa_14_R820/RG2/RG2_openCloseJoint')
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    target_positions = [Vector([0.6, -0.2, 0.5]),
                        Vector([0.6, 0.1, 0.5]),
                        Vector([0.6, -0.1, 0.5]),
                        Vector([0.2, -0.55, 0.8]),
                        Vector([0.3, 0.3, 0.5]),
                        Vector([-0.3, -0.3, 0.5])]
    target_orientations = [Euler([-np.pi / 2, 0, -np.pi / 2]),
                           Euler([-np.pi / 2, 0, -np.pi / 2]),
                           Euler([-np.pi / 2, 0, -np.pi / 2]),
                           Euler([-np.pi, 0, 0]),
                           Euler([0, 0, 0]),
                           Euler([np.pi/2, 0, 0])]

    q0 = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, -0.1])

    robot.moveAbsJ(q_target=q0)

    for i in range(6):
        robot.moveJ(target_position=target_positions[i],
                    target_orientation=target_orientations[i], precision='last')

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikine()

