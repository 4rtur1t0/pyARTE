#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/abb_irb140.ttt scene before using this class.

RobotABBIRB140 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.seriallink import SerialRobot
from artelib.tools import buildT, normalize_angle
from robots.robot import Robot


# def normalize(eul):
#     e = []
#     for i in range(len(eul)):
#         e.append(np.arctan2(np.sin(eul[i]), np.cos(eul[i])))
#     return e


class RobotABBIRB140(Robot):
    def __init__(self, clientID, wheeljoints, armjoints, base, gripper, end_effector, target, camera):
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180])
        max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges (an
        joint_ranges = np.array([[-180, -90, -230, -200, -360, -400],
                                 [180,   110,  50,  200,  115, 400]])
        joint_ranges = joint_ranges * np.pi / 180.0
        self.max_iterations_inverse_kinematics = 15000
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.ikmethod = 'closed-equations'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True

        # DH parameters of the robot
        # robot.name = 'ABB_IRB140_M2000';
        # robot.DH.theta = '[q(1) q(2)-pi/2 q(3) q(4) q(5) q(6)+pi]';
        # robot.DH.d = '[0.352 0 0 0.380 0 0.065]';
        # robot.DH.a = '[0.070 0.360 0 0 0 0]';
        # robot.DH.alpha = '[-pi/2 0 -pi/2 pi/2 -pi/2 0]';
        # robot.J = [];
        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), TCP=np.eye(4), name='ABBIRB140')
        self.serialrobot.append(th=0, d=0.352, a=0.07, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=-np.pi/2, d=0, a=0.36, alpha=0, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0.38, a=0, alpha=np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=np.pi, d=0.065, a=0, alpha=0, link_type='R')

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

    def inversekinematics(self, target_position, target_orientation, q0=None):
        """
        Inverse kinematic method for the ABB IRB140 robot.
        Please, beware that the ABB robot corresponds to a modified version of the original robot that is included in
        Coppelia. In particular, the movement direction of joint2 and joint3 have been reversed and now match the
        positive direction specified by the manufacturer.
        """
        Ttarget = buildT(target_position, target_orientation)
        Ttarget = HomogeneousMatrix(Ttarget)

        L6 = self.serialrobot.transformations[5].d

        # Position of the end effector
        P = Ttarget.pos()
        # z6=z5
        z6 = np.array(Ttarget.array[0:3, 2])
        # Pm = np.array([Px, Py, Pz]).T - L6*W
        Pm = P.T - L6 * z6.T

        # if q(1) is a solution, then q(1) + pi is also a solution
        q1 = np.arctan2(Pm[1], Pm[0])

        # solve for q2, q3
        q2_1, q3_1 = self.solve_for_theta23(q1, Pm)
        q2_2, q3_2 = self.solve_for_theta23(q1 + np.pi, Pm)

        q = np.array([[q1,   q1,         q1,        q1,       q1 + np.pi,   q1 + np.pi,   q1 + np.pi,   q1 + np.pi],
             [q2_1[0],   q2_1[0],    q2_1[1],   q2_1[1],   q2_2[0],       q2_2[0],       q2_2[1],      q2_2[1]],
             [q3_1[0],   q3_1[0],    q3_1[1],   q3_1[1],   q3_2[0],       q3_2[0],       q3_2[1],      q3_2[1]],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0],
             [0, 0, 0, 0, 0, 0, 0, 0]])

        q = np.real(q)

        for i in range(0, 8, 2):
            qi = q[0:6, i]
            # Ti = self.serialrobot.directkinematics(qi)
            # print(Ti)
            w1, w2 = self.solve_spherical_wrist(qi, Ttarget)
            q[3:6, i] = w1
            q[3:6, i+1] = w2

        # % normalize         q         to[-pi, pi]
        for i in range(8):
            q[:, i] = normalize_angle(q[:, i])
        return q

    def solve_for_theta23(self, q1, Pm):
        # See arm geometry
        L2 = self.serialrobot.transformations[1].a
        L3 = self.serialrobot.transformations[3].d
        A01 = self.serialrobot.transformations[0].dh(q1)
        Pm = np.concatenate((Pm, [1]), axis=0)
        # % Express     Pm in the     reference     system     1,    for convenience
        p1 = np.dot(A01.inv().toarray(), Pm.T)
        r = np.linalg.norm(np.array([p1[0], p1[1]]))
        beta = np.arctan2(-p1[1], p1[0])

        a = (L2**2 + r**2 - L3**2) / (2 * r *L2)
        b = (L2**2 + L3**2 - r**2) / (2 * L2 * L3)

        if np.abs(a) < 1.0:
            gamma = np.arccos(a)
        else:
            print('ERROR IN CURRENT SOLUTION: CANNOT COMPUTE INVERSE KINEMATICS FOR THE ABB IRB140 ROBOT. The point is out of the workspace')
            gamma = np.nan

        if np.abs(b) < 1.0:
            eta = np.arccos(b)
        else:
            print('ERROR IN CURRENT SOLUTION: CANNOT COMPUTE INVERSE KINEMATICS FOR THE ABB IRB140 ROBOT. The point is out of the workspace')
            eta = np.nan
        # elbow  up
        q2_1 = np.pi / 2 - beta - gamma
        q3_1 = np.pi / 2 - eta
        # elbow  down
        q2_2 = np.pi / 2 - beta + gamma
        q3_2 = eta - 3 * np.pi / 2
        return np.array([q2_1, q2_2]), np.array([q3_1, q3_2])

    def solve_spherical_wrist(self, q, T):
        """
        Solve robot's wrist using an algebraic solution
        % [sin(q4) * sin(q6) - cos(q4) * cos(q5) * cos(q6), cos(q6) * sin(q4) + cos(q4) * cos(q5) * sin(q6),
           -cos(q4) * sin(q5)]
        % [- cos(q4) * sin(q6) - cos(q5) * cos(q6) * sin(q4), cos(q5) * sin(q4) * sin(q6) - cos(q4) * cos(q6),
           -sin(q4) * sin(q5)]
        % [-cos(q6) * sin(q5), sin(q5) * sin(q6), cos(q5)]

        % degenerate
        % [-cos(q4 + q6), sin(q4 + q6), 0, 0]
        % [-sin(q4 + q6), -cos(q4 + q6), 0, 0]
        % [0, 0, 1, 89 / 200]
        % [0, 0, 0, 1]
        """
        A01 = self.serialrobot.transformations[0].dh(q[0])
        A12 = self.serialrobot.transformations[1].dh(q[1])
        A23 = self.serialrobot.transformations[2].dh(q[2])
        # this allows to compute the value of A34*A45*A56
        Q = A23.inv()*A12.inv()*A01.inv()*T

        # detect the degenerate case when q(5) = 0, this leads to zeros   % in Q13, Q23, Q31 and Q32 and Q33 = 1
        thresh = 1e-6

        # thresh = 0.00001
        # estandar solution
        if 1 - abs(Q[2, 2]) > thresh:
            q5 = np.arccos(Q[2, 2])
            # alternate solution -q5
            q5_ = -q5
            s5 = np.sign(q5)
            s5_ = np.sign(q5_)
            q4 = np.arctan2(-s5 * Q[1, 2], -s5 * Q[0, 2])
            q4_ = np.arctan2(-s5_ * Q[1, 2], -s5_ * Q[0, 2])
            q6 = np.arctan2(s5 * Q[2, 1], -s5 * Q[2, 0])
            q6_ = np.arctan2(s5_ * Q[2, 1], -s5_ * Q[2, 0])
        else:
            # degenerate solution
            q5 = np.real(np.arccos(Q[2, 2]))
            q5_ = q5
            q4 = 0
            q4_ = np.pi
            q6 = np.arctan2(Q[0, 1], -Q[1, 1])
            q6_ = q6 - np.pi
        # two alternate solutions are found
        wrist1 = [q4, q5, q6]
        wrist2 = [q4_, q5_, q6_]
        return np.array(wrist1), np.array(wrist2)




