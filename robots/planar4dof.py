#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/4dofplanar.ttt scene before using this class.

Planar4DOF is a derived class of the Robot base class that particularizes some details of the robot

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from robots.robot import Robot
from kinematics.kinematics_planar4dof import eval_symbolic_jacobian_planar_4dof, eval_symbolic_T_planar4dof


class Planar4DOF(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera):
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180])
        max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-90, -90, -90, -90],
                                 [90,   90,  90,  90]])
        joint_ranges = joint_ranges * np.pi / 180.0
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        # self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        self.ikmethod = 'transpose'

        Robot.__init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera,
                       max_joint_speeds=max_joint_speeds, joint_ranges=joint_ranges)

    def open_gripper(self, precision=False):
        self.gripper.open_gripper(precision=precision)
        if precision:
            self.wait(10)

    def close_gripper(self, precision=False):
        self.gripper.close_gripper(precision=precision)
        if precision:
            self.wait(10)

    def get_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_planar_4dof(q)
        return J, Jv, Jw

    def direct_kinematics(self, q):
        T = eval_symbolic_T_planar4dof(q)
        return T

