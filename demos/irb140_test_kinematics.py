#!/usr/bin/env python
# encoding: utf-8
"""
This script does not need a Coppelia Scene.

The script tests the inverse kinematics method of the ABB IRB140 robot.

M different tests on random joint positions are carried out.

A set of q is generated randomly and uniformly distributed on the joint ranges. For each qi, the position/orientation
of th end effector T is computed. Next, the inverse kinematics is invoked. Given T, the inverse kinematic method returns:
- 8 different solutions (normal case). extended = False
- approx 16 different solutions (normal case). extended = True. This includes alternate solutions for q4 and q6

The script checks that every solution found by the inverse kinematics yields the same matrix T.
 Also, the script checks that the original q is included in the solutions found.

CAUTION: This script uses directly the inverse kinematic method of the
ABBIRB140 robot instead of the typical movement functions moveJ, moveL... etc.

No connection is made with Coppelia in order to move any robot.

@Authors: Arturo Gil
@Time: April 2023
"""
import numpy as np
from artelib.path_planning import random_q
from robots.abbirb140 import RobotABBIRB140


def check_solutions1(robot, qinv, T):
    """
    this tests that every q in qinv yields the same matrix T.
    """
    n_solutions = qinv.shape[1]
    for i in range(n_solutions):
        Ti = robot.directkinematics(qinv[:, i])
        k = np.linalg.norm(T.array-Ti.array)
        if k > 0.01:
            return 1
    return 0


def check_solutions2(qinv, q):
    """
    This checks that, at least, q is included in qinv
    """
    n_solutions = qinv.shape[1]
    q_rep = np.tile(q.T, (n_solutions, 1))
    # print(qinv-q_rep.T)
    m = np.linalg.norm(qinv - q_rep.T, axis=0)
    i = np.argwhere(m < 0.01)
    if len(i) == 0:
        return 1
    return 0


if __name__ == "__main__":
    M = 5000
    robot = RobotABBIRB140(clientID=None)

    for i in range(M):
        q = random_q(robot)
        T = robot.directkinematics(q)
        p = T.pos()
        # exclude singularities
        if np.abs(q[4]) < 0.01 or np.linalg.norm(p[0:1]) < 0.01:
            continue
        qinv = robot.inversekinematics(T.pos(), T.R(), extended=True)
        error1 = check_solutions1(robot, qinv, T)
        if error1:
            print('ERROR FOUND IN INVERSE KINEMATICS: ONE OF THE SOLUTIONS DOES NOT YIELD T')
            print(q)
            exit()
        error2 = check_solutions2(qinv, q)
        if error2:
            print('ERROR FOUND IN INVERSE KINEMATICS: THE ORIGINAL q IS NOT FOUND IN THE SOLUTIONS')
            print(q)
            exit()
    print('FINISHED: NO ERRORS FOUND!')


