#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before running this script.

The demo presents a series of target points and uses robot.inversekinematics_line to follow the targets along a
line in task space.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.euler import Euler
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
    target_positions = [[0.2, -0.45, 0.4],
                        [0.6, -0.2, 0.25]]
    target_orientations = [[-np.pi, 0, 0],
                           [-np.pi/2, 0, -np.pi/2]]
    q = np.array([-np.pi/4, -np.pi/8, np.pi/2, 0.1, 0.1, 0.1])
    robot.set_joint_target_positions(q, precision=True)
    for i in range(len(target_positions)):
        # robot.set_target_position_orientation(target_positions[i], target_orientations[i])
        q_path = robot.inversekinematics_line(target_position=target_positions[i],
                                              target_orientation=Euler(target_orientations[i]), q0=q)
        robot.set_joint_target_trajectory(q_path=q_path, precision='last')
        q = q_path[-1]

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikineline()