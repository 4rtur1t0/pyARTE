#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

    EXERCISE: MOVE THE ROBOT ARBITRARILY IN THE NULL SPACE, considering
    n=6DOF
    m=3DOF, that is, reduce the task space and move the robot to keep the position
     of the end effector constant

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

from robots.abbirb140 import RobotABBIRB140
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def null_space(J, column):
    """
    Obtain a unit vector in the direction
    """
    u, s, vh = np.linalg.svd(J, full_matrices=True)
    qd = vh.T[:, column]
    return qd

def move_along(column):
    n_movements_in_null_space = 30
    for i in range(0, n_movements_in_null_space):
        q = robot.get_joint_positions()
        print('Movement number: ', i)
        J, Jv, Jw = robot.manipulator_jacobian(q)
        # reduce J to meet task
        J = J[0:3, :]
        # obtain column i
        # column = 5
        qd = null_space(J, column)
        if qd[column] < 0:
            qd = -qd
        robot.set_joint_target_velocities(qd)
        robot.wait()


def move_null_space(robot):
    # initial arm position
    q0 = np.array([-np.pi / 8, np.pi/8, np.pi/8, 0.5, 0.5, 0.5])
    robot.moveAbsJ(q_target=q0, precision=True)
    # moves along vector 6, 5, 4 in nulls space
    move_along(5)
    robot.moveAbsJ(q_target=q0, precision=True)
    move_along(4)
    robot.moveAbsJ(q_target=q0, precision=True)
    move_along(3)




if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    move_null_space(robot)
    simulation.stop()
    robot.plot_trajectories()
