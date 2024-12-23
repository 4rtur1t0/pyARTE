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
from robots.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


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
    # EJERCICIO: INTERPOLE POSICIONES Y ORIENTACIONES PARA IR DE PA/OA A PB/OB
    # CUIDADO: Debe expresar las posiciones como Vector(np.array[x, y, z])
    # CUIDADO: Debe expresar las orientaciones como Euler(np.array[alpha, beta, gamma])


    return positions, orientations


def path_in_workspace():
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation)
    robot.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))


    pA = Vector([0.5, -0.5, 0.9])
    oA = Euler([0, 0, 0])
    pB = Vector([0.5, 0.5, 0.4])
    oB = Euler([-np.pi / 2, np.pi / 2, -np.pi / 2])

    tps, tos = path_planning_line(pA, oA, pB, oB)

    # ESTA FUNCIÓN MUESTRA TODAS LAS POSICIONES Y ORIENTACIONES INTERPOLADAS
    frame.show_target_points(target_positions=tps, target_orientations=tos, wait_time=0.5)

    # FINALMENTE, INTENTE ALCANZAR LAS POSICIONES Y ORIENTACIONES CON EL ROBOT
    # USE MOVEJ

    simulation.wait(50)
    simulation.stop()


if __name__ == "__main__":
    path_in_workspace()
