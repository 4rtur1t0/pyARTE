#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/more/youbot.ttt scene before running this script.

@Authors: Arturo Gil
@Time: December 2023
"""
import numpy as np
from robots.youbot import YouBotArm


def check_kinematics():
    robot = YouBotArm(simulation=None)

    q = np.array([0.1, 0.1, 0.1, 0.1, 0.1])
    T = robot.directkinematics(q)

    qinv = robot.inversekinematics(T)

    for i in range(3):
        Ti = robot.directkinematics(qinv[i])
        print(Ti)


if __name__ == "__main__":
    check_kinematics()

