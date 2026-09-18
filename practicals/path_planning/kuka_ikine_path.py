#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_lbr_simple.ttt scene before running this script.

The demo presents a series of target points and uses robot.inversekinematics_line to follow the targets along a
line in task space.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

from artelib.euler import Euler
from artelib.plottools import plot
from artelib.tools import euler2q, euler2rot, w_lateral
from robots.grippers import GripperRG2
from robots.kukalbr import RobotKUKALBR
from robots.simulation import Simulation


def ikineline():
    simulation = Simulation()
    simulation.start()
    robot = RobotKUKALBR(simulation=simulation)
    robot.start()
    gripper = GripperRG2(simulation=simulation)
    gripper.start()
    target_positions = [[0.4, -0.4, 0.5],
                        [0.41, -0.4, 0.5],
                        [0.42, -0.4, 0.5],
                        [0.43, -0.4, 0.5],
                        [0.44, -0.4, 0.5],
                        [0.45, -0.4, 0.5],
                        [0.46, -0.4, 0.5],
                        [0.47, -0.4, 0.5],
                        [0.4, 0.4, 0.5],
                        [0.4, -0.4, 0.5],
                        [0.4, 0.4, 0.5],
                        [0.4, -0.4, 0.5],
                        [0.4, 0.4, 0.5],
                        [0.4, -0.4, 0.5],
                        [0.4, 0.4, 0.5],
                        [0.4, -0.4, 0.5],
                        [0.4, 0.4, 0.5]]

    target_orientations = [[0, np.pi/2, 0],
                           [0, np.pi / 2, 0],
                           [0, np.pi / 2, 0],
                           [0, np.pi / 2, 0],
                           [0, np.pi / 2, 0],
                           [0, np.pi / 2, 0],
                           [0, np.pi / 2, 0],
                           [0, np.pi / 2, 0],
                           [np.pi/2, 0, 0], # ok
                           [-np.pi/2, 0, 0], # ok
                           [-np.pi/2, 0, 0], #ok
                           [0, np.pi/2, np.pi/2],
                           [0, 0, np.pi/2],
                           [0, 0, -np.pi/2],
                           [0, np.pi/2, 0],
                           [np.pi/2, np.pi/2, np.pi/2],
                           [np.pi/2, np.pi/2, -np.pi/2]]

    q = np.array([-np.pi/4, 0, 0, -np.pi/2, 0.1, 0.1, 0])
    robot.moveAbsJ(q, precision=True)
    w = []
    for i in range(len(target_positions)):
        # robot.set_target_position_orientation(target_positions[i], target_orientations[i])
        q_path = robot.inversekinematics_line(target_position=target_positions[i],
                                              target_orientation=Euler(target_orientations[i]), q0=q)
        robot.moveAbsPath(q_path=q_path, precision='last')
        q = q_path[-1]

    # show the values of the secondary function w
    qt = robot.get_trajectories()
    for q in qt:
        wi = w_lateral(q, qmax=robot.joint_ranges[1], qmin=robot.joint_ranges[0])
        w.append(wi)
    plot(w)

    simulation.stop()
    robot.plot_trajectories()


if __name__ == "__main__":
    ikineline()