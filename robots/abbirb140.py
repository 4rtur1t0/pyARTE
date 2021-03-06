#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/abb_irb140.ttt scene before using this class.

RobotABBIRB140 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from robots.robot import Robot


class RobotABBIRB140(Robot):
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


