#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/kuka_lbr_14_R820.ttt scene before using this class.

RobotKUKALBR is a derived class of the Robot base class that particularizes some details of the robot

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import delta_q
from artelib.seriallink import SerialRobot
from artelib.tools import compute_kinematic_errors, minimize_w_lateral
from robots.robot import Robot
from kinematics.kinematics_kukalbr import eval_symbolic_jacobian_KUKALBR


class RobotKUKALBR(Robot):
    def __init__(self, simulation):
        # init base class attributes
        Robot.__init__(self, simulation=simulation)
        # self.clientID = clientID
        self.DOF = 7
        self.q_current = np.zeros((1, self.DOF))
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180, 180])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-180, -180, -180, -180, -180, -180, -180],
                                 [180,   180,  180,  180,  180,  180,  180]])
        self.joint_ranges = joint_ranges * np.pi / 180.0
        # max errors during computation of inverse kinematics
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.max_iterations_inverse_kinematics = 1500
        self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        # self.ikmethod = 'transpose'
        self.epsilonq = 0.001
        # whether joint limits should be applied during inverse kinematics
        self.do_apply_joint_limits = True
        self.secondary_objective = True
        self.do_apply_joint_limits = None
        # wait these iterations before a WARNING is issued
        self.max_iterations_joint_target = 100

        self.serialrobot = SerialRobot(n=7, T0=np.eye(4), name='KUKALBR')
        self.serialrobot.append(th=0, d=0.36,  a=0, alpha=-np.pi/2, link_type='R')
        self.serialrobot.append(th=0, d=0,     a=0, alpha=np.pi/2, link_type='R')
        self.serialrobot.append(th=0, d=0.420, a=0, alpha=np.pi/2, link_type='R')
        self.serialrobot.append(th=0, d=0,     a=0, alpha=-np.pi/2, link_type='R')
        self.serialrobot.append(th=0, d=0.4,   a=0, alpha=-np.pi/2, link_type='R')
        self.serialrobot.append(th=0, d=0,     a=0, alpha=np.pi/2, link_type='R')
        self.serialrobot.append(th=0, d=0.111, a=0, alpha=0)

    def start(self, base_name='/LBR_iiwa_14_R820', joint_name='joint'):
        robotbase = self.simulation.sim.getObject(base_name)
        q1 = self.simulation.sim.getObject(base_name + '/' + joint_name + '1')
        q2 = self.simulation.sim.getObject(base_name + '/' + joint_name + '2')
        q3 = self.simulation.sim.getObject(base_name + '/' + joint_name + '3')
        q4 = self.simulation.sim.getObject(base_name + '/' + joint_name + '4')
        q5 = self.simulation.sim.getObject(base_name + '/' + joint_name + '5')
        q6 = self.simulation.sim.getObject(base_name + '/' + joint_name + '6')
        q7 = self.simulation.sim.getObject(base_name + '/' + joint_name + '7')

        joints = []
        joints.append(q1)
        joints.append(q2)
        joints.append(q3)
        joints.append(q4)
        joints.append(q5)
        joints.append(q6)
        joints.append(q7)
        self.joints = joints

    def get_symbolic_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_KUKALBR(q)
        return J, Jv, Jw

    def inversekinematics(self, target_position, target_orientation, q0, extended=None):
        """
        This a particular inverse kinematics function for this robot.
        If self.secondary is set, then the null space is used to find
        """
        # build transform using position and Quaternion
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        q = q0
        qmin = self.joint_ranges[0]
        qmax = self.joint_ranges[1]
        for i in range(0, self.max_iterations_inverse_kinematics):
            print('Iteration number: ', i)
            Ti = self.directkinematics(q)
            e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
            print('e: ', e)
            print('errordist, error orient: ', error_dist, error_orient)
            if error_dist < self.max_error_dist_inversekinematics and error_orient < self.max_error_orient_inversekinematics:
                print('Converged!!')
                break
            J, Jv, Jw = self.manipulator_jacobian(q)
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
