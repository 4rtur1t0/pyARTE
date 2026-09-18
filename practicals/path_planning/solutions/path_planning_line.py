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
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation

def path_planning_line(pA, oA, pB, oB, n):
    pA = pA.array
    pB = pB.array
    oA = oA.abg
    oB = oB.abg
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
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))

    # initial and final target points
    n = 50 # number of points on the line
    pA = Vector([0.5, -0.5, 0.9])
    oA = Euler([0, 0, 0])
    pB = Vector([0.5, 0.5, 0.4])
    oB = Euler([-np.pi / 2, np.pi / 2, -np.pi / 2])

    tps, tos = path_planning_line(pA, oA, pB, oB, n)
    # show the target points on Coppelia
    print('SHOWING TARGET POINTS ON COPPELIA!!')
    frame.show_target_points(target_positions=tps, target_orientations=tos)

    # prueba con diferentes posiciones articulare iniciales
    # robot.moveAbsJ(q_target=[-np.pi / 4, np.pi / 2, -np.pi, 0, 0, 0])
    robot.moveAbsJ(q_target=[0, 0, 0, 0, 0, 0])
    for i in range(len(tps)):
        robot.moveJ(target_position=tps[i],
                    target_orientation=tos[i])

    simulation.stop()


if __name__ == "__main__":
    path_in_workspace()
