#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_lbr_14_R820.ttt scene before using this class.

RobotKUKALBR is a derived class of the Robot base class that particularizes some details of the robot

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

from artelib import homogeneousmatrix
from artelib.inverse_kinematics import delta_q
from artelib.path_planning import generate_target_positions, generate_target_orientations_Q
from artelib.tools import compute_kinematic_errors, buildT, rot2quaternion, minimize_w_central, minimize_w_lateral, \
    w_lateral
from robots.robot import Robot
from kinematics.kinematics_kukalbr import eval_symbolic_jacobian_KUKALBR, eval_symbolic_T_KUKALBR


DELTA_TIME = 50.0/1000.0


class RobotKUKALBR(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera):
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180, 180])
        max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-180, -180, -180, -180, -180, -180, -180],
                                 [180,   180, 180,  180,  180,  180, 180]])
        joint_ranges = joint_ranges * np.pi / 180.0

        # max errors during computation of inverse kinematics
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01

        self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        # self.ikmethod = 'transpose'
        # whether joint limits should be applied during inverse kinematics
        self.do_apply_joint_limits = True
        self.secondary_objective = True

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
        J, Jv, Jw = eval_symbolic_jacobian_KUKALBR(q)
        return J, Jv, Jw

    def direct_kinematics(self, q):
        T = eval_symbolic_T_KUKALBR(q)
        return homogeneousmatrix.HomogeneousMatrix(T)

    def inversekinematics(self, target_position, target_orientation, q0):
        """
        This a particular inverse kinematics function for this robot.
        If self.secondary is set, then the null space is used to find
        """
        # build transform using position and Quaternion
        Ttarget = buildT(target_position, target_orientation)
        q = q0
        error_dists = []
        error_orients = []
        es = []
        qmin = self.joint_ranges[0]
        qmax = self.joint_ranges[1]
        for i in range(0, self.max_iterations_inverse_kinematics):
            print('Iteration number: ', i)
            Ti = self.direct_kinematics(q)
            e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
            error_dists.append(np.linalg.norm(error_dist))
            error_orients.append(np.linalg.norm(error_orient))
            print('e: ', e)
            es.append(e)
            print('errordist, error orient: ', error_dist, error_orient)
            if error_dist < self.max_error_dist_inversekinematics and error_orient < self.max_error_orient_inversekinematics:
                print('Converged!!')
                break
            J, Jv, Jw = self.get_jacobian(q)
            qda = delta_q(J, e, method=self.ikmethod)
            if self.secondary_objective:
                # qdb = minimize_w_central(J, q, qc, K)
                qdb = minimize_w_lateral(J, q, qmax=qmax, qmin=qmin)
                qdb = 0.5 * np.linalg.norm(qda) * qdb
                q = q + qda + qdb
            else:
                q = q + qda
            # apply joint limits if selected
            if self.do_apply_joint_limits:
                [q, _] = self.apply_joint_limits(q)
        return q

