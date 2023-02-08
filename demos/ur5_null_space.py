#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

The script provides an example to move a UR5 robot along the null space.

The active space can be defined by selecting the rows of the Manipulator Jacobian.
The column in the V matrix can be selected, in this sense, one can move the robot along the different
vectors that form the basis of the null space.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
# move on the null space (with vh, column 6)
from robots.grippers import GripperRG2
from robots.simulation import Simulation
from robots.ur5 import RobotUR5


def null_space_along(robot, m, col, n_steps=20):
    """
    Define the task (m)
    Define the direction of movement (column of V)
    Define the number of movements (n_steps)
    """
    q_path = []
    q = robot.get_joint_positions()
    for i in range(0, n_steps):
        J, Jv, Jw = robot.get_jacobian(q)
        Jr = J[0:m, :]
        u, s, vh = np.linalg.svd(Jr, full_matrices=True)
        # caution, obtaining v transposed --> must transpose
        qd = vh.T[:, col]
        q = q + 0.05*qd
        q_path.append(q)
        # robot.set_joint_target_positions(q, precision=True)
    robot.set_joint_target_trajectory(q_path)


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotUR5(clientID=clientID)
    robot.start()
    gripper = GripperRG2(clientID=clientID)
    gripper.start()

    q0 = np.pi / 8 * np.array([-6, 1, 3, 1, 2, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    null_space_along(robot, m=3, col=3)
    null_space_along(robot, m=3, col=4)
    null_space_along(robot, m=3, col=5)

    robot.plot_trajectories()
    simulation.stop()

