#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_14_R820.ttt scene before running this script.

    EXERCISE: MOVE ARBITRARILY THE ROBOT IN THE NULL SPACE CONSIDERING
    n=7 DOF
    m=6 DOF, task in 3+3 dimensions

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.plottools import plot_vars
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def null_space(J):
    """
    Obtain a unit vector in the direction
    """
    u, s, vh = np.linalg.svd(J, full_matrices=True)
    qd = vh.T[:, 6]
    return qd


def move_null_space(robot):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, -np.pi / 2, 0.1, 0.1, 0.1])
    robot.moveAbsJ(q_target=q0, precision=True)
    n_movements_in_null_space = 500

    for i in range(0, n_movements_in_null_space):
        q = robot.get_joint_positions()
        print('Movement number: ', i)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        qd = null_space(J)
        if qd[2] < 0:
            qd = -qd
        robot.set_joint_target_velocities(qd)
        robot.wait()


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotKUKALBR(simulation=simulation)
    robot.start()
    move_null_space(robot)

    simulation.stop()
    robot.plot_trajectories()
