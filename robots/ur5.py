#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

RobotUR5 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib import homogeneousmatrix
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.seriallink import SerialRobot
from robots.robot import Robot
from kinematics.kinematics_ur5 import eval_symbolic_jacobian_UR5, eval_symbolic_T_UR5


class RobotUR5(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera):
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180])
        max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-360, -360, -360, -360, -360, -360],
                                 [360,   360,  360,  360,  360,  360]])
        joint_ranges = joint_ranges * np.pi / 180.0
        self.max_iterations_inverse_kinematics = 15000
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        # self.ikmethod = 'transpose'
        self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True

        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), TCP=np.eye(4), name='UR5')
        self.serialrobot.append(th=-np.pi/2, d=0.089159, a=0, alpha=np.pi/2)
        self.serialrobot.append(th=+np.pi/2, d=0,        a=0.425, alpha=0)
        self.serialrobot.append(th=0,        d=0,        a=0.39225, alpha=0)
        self.serialrobot.append(th=-np.pi/2, d=0.10915,  a=0, alpha=-np.pi/2)
        self.serialrobot.append(th=0,        d=0.09465,  a=0, alpha=np.pi/2)
        self.serialrobot.append(th=0,        d=0.0823,   a=0, alpha=0)

        Robot.__init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target,
                       max_joint_speeds=max_joint_speeds, joint_ranges=joint_ranges, camera=camera)

    def open_gripper(self, precision=False):
        self.gripper.open_gripper(precision=precision)
        if precision:
            self.wait(10)

    def close_gripper(self, precision=False):
        self.gripper.close_gripper(precision=precision)
        if precision:
            self.wait(10)

    def get_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_UR5(q)
        return J, Jv, Jw

    def directkinematics(self, q):
        # T1 = eval_symbolic_T_UR5(q)
        # T1 = HomogeneousMatrix(T1)
        T = self.serialrobot.directkinematics(q)
        return homogeneousmatrix.HomogeneousMatrix(T)
