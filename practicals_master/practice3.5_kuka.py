#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka.ttt scene before running this script.

The script provides an example to move a KUKA LBR robot along the null space.

The active space can be defined by selecting the rows of the Manipulator Jacobian.
The column in the V matrix can be selected, in this sense, one can move the robot along the different
vectors that form the basis of the null space.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


# move on the null space (with vh, column 6)
def null_space_along(robot, m, col, indexpos, n_steps=30):
    """
    Define the task (m)
    Define the direction of movement (column of V)
    Define the number of movements (n_steps)
    """
    q_path = []
    q = robot.get_joint_positions()
    for i in range(n_steps):
        J, Jv, Jw = robot.manipulator_jacobian(q)
        Jr = J[0:m, :]
        u, s, vh = np.linalg.svd(Jr, full_matrices=True)
        # caution, obtaining v transposed --> must transpose
        qd = vh.T[:, col]
        if qd[indexpos] > 0:
            qd = -qd
        q = q + 0.05*qd
        q_path.append(q)
    robot.set_joint_target_trajectory(q_path, precision='last')
    robot.plot_trajectories()


if __name__ == "__main__":
    simulation = Simulation()
    clientID = simulation.start()
    robot = RobotKUKALBR(clientID=clientID)
    robot.start()
    gripper = GripperRG2(clientID=clientID)
    gripper.start()
    q0 = np.pi / 8 * np.array([-3, 3, 3, -3, 2, 2, 1])
    # set initial position of robot
    robot.set_joint_target_positions(q0, precision=True)
    null_space_along(robot, m=3, col=3, indexpos=2)
    robot.set_joint_target_positions(q0, precision=True)
    null_space_along(robot, m=3, col=4, indexpos=4)
    robot.set_joint_target_positions(q0, precision=True)
    null_space_along(robot, m=3, col=5, indexpos=2)
    robot.set_joint_target_positions(q0, precision=True)
    null_space_along(robot, m=3, col=6, indexpos=2)

    simulation.stop()

