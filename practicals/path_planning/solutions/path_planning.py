#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

@Authors: Arturo Gil
@Time: April 2022
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.vector import Vector
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def show_target_points(clientID, target_positions, target_orientations, wait_time=10):
    frame = ReferenceFrame(clientID=clientID)
    frame.start()
    for i in range(len(target_positions)):
        T = HomogeneousMatrix(target_positions[i], target_orientations[i])
        frame.set_position_and_orientation(T)
        frame.wait(wait_time)


def n_movements_pos(pA, pB):
    """
    Computes the number of waypoints along a line when moving the end effector at a constant linear speed.
    """
    vmax = 1  # m/s
    delta_time = 0.05  # 50 ms
    total_time = np.linalg.norm(np.array(pB) - np.array(pA)) / vmax
    n = total_time / delta_time
    n = np.ceil(n)
    return int(n)


def n_movements_orient(eA, eB):
    """
    Computes the number of orientations needed to change the orientations from Euler angles eA to eB at constant angular
    speed. This is a rough approximation and the library uses quaternions for this purpose.
    """
    wmax = 1  # rad/s
    delta_time = 0.05  # 50 ms
    total_time = np.linalg.norm(np.array(eB) - np.array(eA)) / wmax
    n = total_time / delta_time
    n = np.ceil(n)
    return int(n)


def path_planning_line(pA, oA, pB, oB):
    pA = pA.array
    pB = pB.array
    oA = oA.abg
    oB = oB.abg
    n1 = n_movements_pos(pB, pA)
    n2 = n_movements_orient(oB, oA)
    n = max(n1, n2)
    t = np.linspace(0, 1, n)
    positions = []
    orientations = []

    for i in range(n):
        p = (1 - t[i]) * pA + t[i] * pB
        o = (1 - t[i]) * oA + t[i] * oB
        positions.append(p)
        orientations.append(Euler(o))
    return positions, orientations


def path_in_workspace():
    simulation = Simulation()
    clientID = simulation.start()

    pA = Vector([0.5, -0.5, 0.9])
    oA = Euler([0, 0, 0])
    pB = Vector([0.5, 0.5, 0.4])
    oB = Euler([-np.pi / 2, np.pi / 2, -np.pi / 2])

    tps, tos = path_planning_line(pA, oA, pB, oB)

    show_target_points(clientID=clientID, target_positions=tps, target_orientations=tos, wait_time=2)

    simulation.wait(50)
    simulation.stop()


if __name__ == "__main__":
    path_in_workspace()
