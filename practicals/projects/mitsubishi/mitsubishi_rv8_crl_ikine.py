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
# from robots.abbirb140 import RobotABBIRB140
# from robots.abbirb52 import RobotMIRB52
from robots.grippers import SuctionPad
from robots.mitsubishi_rv8_crl import RobotMitsubishiRV8CRL
from robots.objects import ReferenceFrame
from robots.simulation import Simulation
from robots.camera import Camera


def ikine():
    simulation = Simulation()
    simulation.start()
    robot = RobotMitsubishiRV8CRL(simulation=simulation)
    robot.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()

    # Initial q
    q = np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5])
    robot.moveAbsJ(q_target=q)

    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.2]), Euler([0, 0, 0])))

    q = robot.get_joint_positions()
    T = robot.directkinematics(q)

    print('ROBOT IS AT: ')
    T.print_nice()
    print('Trying to reach T:')
    qinv = robot.inversekinematics(target_position=T.pos(),
                                   target_orientation=T.R())

    for i in range(qinv.shape[1]):
        Ti = robot.directkinematics(qinv[:, i])
        print('INVERSE KINEMATICS SOLUTION')
        print(qinv[:, i])
        Ti.print_nice()
        # move the robot!
        robot.moveAbsJ(q_target=qinv[:, i])
        frame.show_target_point(target_position=T.pos(),
                                target_orientation=T.R())

    simulation.stop()


if __name__ == "__main__":
    ikine()


