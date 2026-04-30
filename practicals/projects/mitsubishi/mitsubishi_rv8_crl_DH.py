#!/usr/bin/env python
# encoding: utf-8
"""
Please open the rv8_crl.ttt scene before running this script.

@Authors: Arturo Gil
@Time: March 2025
"""
import numpy as np
from robots.mitsubishi_rv8_crl import RobotMitsubishiRV8CRL
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def find_transforms():
    simulation = Simulation()
    simulation.start()
    robot = RobotMitsubishiRV8CRL(simulation=simulation)
    # robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # Posicion inicial del robot
    q = np.array([0, 0, 0, 0, 0, 0])

    # Calculamos las transformaciones de DH del robot
    A01 = robot.dh(q, 0)
    A12 = robot.dh(q, 1)
    A23 = robot.dh(q, 2)
    A34 = robot.dh(q, 3)
    A45 = robot.dh(q, 4)
    A56 = robot.dh(q, 5)

    # Las mostramos por pantalla
    A01.print_nice()
    A12.print_nice()
    A23.print_nice()
    A34.print_nice()
    A45.print_nice()
    A56.print_nice()

    ##############################################################################################
    # TAREA: Se deben hallar las transformaciones para colocar todas las articulaciones (ejes)
    # del robot y todos los eslabones.
    ##############################################################################################

    simulation.stop()


if __name__ == "__main__":
    find_transforms()


