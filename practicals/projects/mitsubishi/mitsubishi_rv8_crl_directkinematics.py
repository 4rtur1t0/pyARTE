#!/usr/bin/env python
# encoding: utf-8
"""
Please open the rv8-crl.ttt scene before running this script.

@Authors: Arturo Gil
@Time: October 2024
"""
import numpy as np
from artelib.euler import Euler
from artelib.vector import Vector
from artelib.homogeneousmatrix import HomogeneousMatrix
from robots.mitsubishi_rv8_crl import RobotMitsubishiRV8CRL
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def directkinematics():
    simulation = Simulation()
    simulation.start()
    robot = RobotMitsubishiRV8CRL(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    # Se configura el TCP de la herramienta
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.2]), Euler([0, 0, 0])))

    q = np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
    robot.moveAbsJ(q_target=q)

    q = robot.get_joint_positions()
    T = robot.directkinematics(q)

    print('ROBOT END EFFECTOR IS AT: ')
    T.print_nice()

    frame.show_target_point(target_position=T.pos(),
                            target_orientation=T.R())

    simulation.wait_time(10)
    simulation.stop()


if __name__ == "__main__":
    directkinematics()


