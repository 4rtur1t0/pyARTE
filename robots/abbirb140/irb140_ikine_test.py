#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/abbirb140.ttt scene before running this script.

The script computes the inverse kinematic of the IRB140 robot and sends joint values to Coppelia to view
the results.

In this script the instructions moveL, moveJ or moveAbsJ are not used, since the objective i to observe
the different solutions to the inversekinematics problem.

@Authors: Arturo Gil
@Time: April 2022
@Revision: July 2023, Arturo Gil
"""
import numpy as np
from artelib.euler import Euler
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.rotationmatrix import RotationMatrix
from artelib.vector import Vector
from robots.abbirb140.abbirb140 import RobotABBIRB140
from robots.objects import ReferenceFrame
from robots.simulation import Simulation


def move_robot(robot, q):
    if q.size > 0:
        n_solutions = q.shape[1]
    else:
        n_solutions = 0
    print('COMANDANDO AL ROBOT A LAS SOLUCIONES')
    for i in range(n_solutions):
        print(i)
        qi = q[:, i]
        robot.moveAbsJ(q_target=qi, precision=True)


def check_q_yield_T(robot, T, qinv):
    n_solutions = qinv.shape[1]
    print('Found n different solutions', n_solutions)
    print('Checking solutions')
    test = True
    # check that every solution yields T
    for i in range(n_solutions):
        Ti = robot.directkinematics(qinv[:, i])
        ep = np.linalg.norm(Ti.pos()-T.pos())
        diff = Ti.Q() - T.Q()
        eo = np.linalg.norm(diff.array)
        if ep > 0.01 or eo > 0.01:
            test = False
    if test:
        robot.error_print.print('TEST1 PASSED', 'green')
        robot.error_print.print('All solutions yield the same T', 'green')
    else:
        robot.error_print.print('TEST1 FAILED', 'red')
        robot.error_print.print('All solutions do not yield the same T')


def check_q_orig_in_qinv(q, qinv):
    n_solutions = qinv.shape[1]
    print('Found n different solutions', n_solutions)
    print('Checking q in qinv')
    # print('q', q)
    #print('qinv', qinv)
    q = q[:, np.newaxis]
    diff = qinv - q
    dists = np.linalg.norm(diff, axis=0)
    idx = np.where(dists < 0.01)
    if len(idx[0]) == 1:
        robot.error_print.print('TEST 2 PASSED: the original q is found in the solutions', 'green')
    else:
        robot.error_print.print('TEST 2 FAILED: the original q is found in the solutions', 'red')


def irb140_ikine1(robot):
    """
    Use a q0 out of any singularity
    """
    # set initial joint position in a q5=0 singularity
    q0 = np.array([-np.pi/2, 0.0, -np.pi/16, np.pi/4, 0.1, -np.pi/4])
    robot.moveAbsJ(q_target=q0, precision=True)
    T = robot.directkinematics(q0)
    qinv = robot.inverse_kinematics(target_position=T.pos(),
                                    target_orientation=T.R(),
                                    extended=False)
    check_q_yield_T(robot, T, qinv)
    check_q_orig_in_qinv(q0, qinv)
    move_robot(robot, qinv)



def irb140_ikine2(robot):
    """
    This test must fail in TEST 2
    """
    # set initial joint position in a q5=0 singularity
    q0 = np.array([0.1, 0.1, 0.1, np.pi/8, 0.0, 0.5])
    robot.moveAbsJ(q_target=q0, precision=True)
    T = robot.directkinematics(q0)
    qinv = robot.inverse_kinematics(target_position=T.pos(),
                                    target_orientation=T.R(),
                                    extended=False)
    check_q_yield_T(robot, T, qinv)
    check_q_orig_in_qinv(q0, qinv)
    move_robot(robot, qinv)


def irb140_ikine3(robot):
    """
    Similar to ikine1 test, but we pass q0 as the previous solution.
    In the singularity, this will not fail test 2
    """
    # set initial joint position in a q5=0 singularity
    q0 = np.array([0.1, 0.1, 0.1, np.pi/8, 0.0, 0.5])
    # q0 = np.array([-np.pi / 2, 0.0, -np.pi / 16, np.pi / 4, 0.0, -np.pi / 4])
    robot.moveAbsJ(q_target=q0, precision=True)
    T = robot.directkinematics(q0)
    qinv = robot.inverse_kinematics(target_position=T.pos(),
                                    target_orientation=T.R(),
                                    extended=True,
                                    q0=q0)
    check_q_yield_T(robot, T, qinv)
    check_q_orig_in_qinv(q0, qinv)
    move_robot(robot, qinv)


if __name__ == "__main__":
    simulation = Simulation()
    simulation.start()
    frame = ReferenceFrame(simulation=simulation)
    frame.start()
    robot = RobotABBIRB140(simulation=simulation, frame=frame)
    robot.start()
    robot.set_TCP(HomogeneousMatrix(Vector([0, 0, 0.195]), RotationMatrix(np.eye(3))))
    # test solutions
    # irb140_ikine1(robot)
    # irb140_ikine2(robot)
    irb140_ikine3(robot)
    # Stop arm and simulation
    simulation.stop()
