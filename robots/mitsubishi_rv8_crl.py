#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before using this class.

RobotABBIRB140 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.path_planning import filter_path, path_trapezoidal_i
from artelib.seriallink import SerialRobot
from robots.robot import Robot
import matplotlib.pyplot as plt


class RobotMitsubishiRV8CRL(Robot):
    def __init__(self, simulation):
        # init base class attributes
        Robot.__init__(self, simulation)
        # self.clientID = clientID
        self.DOF = 6
        self.q_current = np.zeros((1, self.DOF))

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([200, 200, 245, 348, 360, 450])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges. joints 4 and 6 can be configured as unlimited
        # default joint limits:
        # q1 (+-180), q2 (-90,110), q3 (-230, 50), q4 (+-200), q5 (+-115), q6 (+-400)
        # here, the joint range for q4 has been extended
        joint_ranges = np.array([[-180, -180, -300, -400, -180, -400],
                                 [180, 180, 180, 400, 180, 400]])
        self.pid_controllers = np.array([[0.8, 0, 0.2],
                                         [0.1, 0, 0.1],
                                         [0.2, 0, 0.1],
                                         [0.1, 0, 0.1],
                                         [0.1, 0, 0.1],
                                         [0.1, 0, 0.1]])
        self.joint_ranges = joint_ranges * np.pi / 180.0
        self.max_iterations_inverse_kinematics = 15000
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.ikmethod = 'closed-equations'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True
        # Sum of squared errors in joints to finish precision=True instructions
        # self.epsilonq = 0.0002
        self.epsilonq = 0.02

        # DH parameters of the robot
        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), name='RV8_CRL')
        self.serialrobot.append(th=0, d=0.39, a=0.0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=-np.pi / 2, d=0, a=0.45, alpha=0, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0.1, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0.47, a=0, alpha=np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=np.pi, d=0.085, a=0, alpha=0, link_type='R')

    def start(self, base_name='/RV8_CRL', joint_name='joint'):
        armjoints = []
        # Get the handles of the relevant objects
        robotbase = self.simulation.sim.getObject(base_name)
        q1 = self.simulation.sim.getObject(base_name + '/' + joint_name + '1')
        q2 = self.simulation.sim.getObject(base_name + '/' + joint_name + '2')
        q3 = self.simulation.sim.getObject(base_name + '/' + joint_name + '3')
        q4 = self.simulation.sim.getObject(base_name + '/' + joint_name + '4')
        q5 = self.simulation.sim.getObject(base_name + '/' + joint_name + '5')
        q6 = self.simulation.sim.getObject(base_name + '/' + joint_name + '6')

        armjoints.append(q1)
        armjoints.append(q2)
        armjoints.append(q3)
        armjoints.append(q4)
        armjoints.append(q5)
        armjoints.append(q6)
        # must store the joints
        self.joints = armjoints

    def inversekinematics(self, target_position, target_orientation, q0=None, extended=False):
        """
        Inverse kinematic method for the ABB IRB140 robot.

        Please, beware that the ABB robot corresponds to a modified version of the original robot that is included in
        Coppelia. In particular, the movement direction of joint2 and joint3 have been reversed and now match the
        positive direction specified by the manufacturer (ABB).

        Generally, given an end effector position and orientation, 8 different solutions are provided for the inverse
        kinematic problem. If the extended option is enabled, some extra solutions are provided. These solutions exist,
        given that the joint ranges for q4 and q6 are [-400, 400] degrees.
        """
        q = []
        Ttarget = HomogeneousMatrix(target_position, target_orientation)

        # Remove Ttcp, so that T_end_effector is specified
        Tcp_inv = self.Ttcp.inv()
        Ttarget = Ttarget * Tcp_inv

        # get the value from the robot class (last link length)
        L6 = self.serialrobot.transformations[5].d

        # Position of the end effector
        P = Ttarget.pos()
        # z6=z5
        z6 = np.array(Ttarget.array[0:3, 2])
        # Compute wrist center point
        Pm = P.T - L6 * z6.T

        # if q(1) is a solution, then q(1) +- pi is also a solution
        q1 = np.arctan2(Pm[1], Pm[0])
        q1_a = q1 + np.pi
        q1_b = q1 - np.pi
        q1_a = np.arctan2(np.sin(q1_a), np.cos(q1_a))
        q1_b = np.arctan2(np.sin(q1_b), np.cos(q1_b))
        q1s = np.array([q1, q1_a, q1_b])
        # Find unique values within 7 decimals
        # this avoids having multiple equal solutions
        q1s, idx = np.unique(q1s.round(decimals=7), axis=0, return_index=True)
        q1s = q1s[idx]
        # for each possible solution in q1, compute q2 and q3
        for i in range(len(q1s)):
            # for each q1 solve for q2, q3. Caution do not normalize q2 or q3
            q2, q3 = self.solve_for_theta23(q1s[i], Pm)
            if np.isnan(np.sum(q2 + q3)):
                continue
            v1 = np.array([q1s[i], q2[0], q3[0]])
            v2 = np.array([q1s[i], q2[1], q3[1]])
            q.append(v1)
            q.append(v2)
        # make them real numbers! and transpose
        q = np.array(q)
        q = q.T
        if len(q) == 0:
            return []
        n_solutions = q.shape[1]
        q_total = []
        # solve the last three joints, for each value for q1, q2 and q3
        for i in range(n_solutions):
            qi = q[:, i]
            # two different orientations with q4 in [-pi, pi] and q6 in [-pi, pi]
            # in the non extended version, two different solutions are provided
            if not extended:
                # append the two solutions to the last three joints
                qwa, qwb = self.solve_spherical_wrist(qi, Ttarget)
                q1 = np.concatenate((qi, qwa), axis=0)
                q2 = np.concatenate((qi, qwb), axis=0)
                q_total.append(q1)
                q_total.append(q2)
            # in the extended version, we consider the extended range in q4 and q6 (adding +-2pi combinations)
            else:
                qwa, qwb = self.solve_spherical_wrist(qi, Ttarget)
                q1 = extend_solutions(qi, qwa)
                q2 = extend_solutions(qi, qwb)
                q_total.extend(q1)
                q_total.extend(q2)
        q_total = np.array(q_total)
        # transpose, solutions are arranged by columns
        q_total = q_total.T
        return q_total

    def solve_for_theta23(self, q1, Pm):
        # See arm geometry
        L2 = self.serialrobot.transformations[1].a
        L3 = self.serialrobot.transformations[2].a
        L4 = self.serialrobot.transformations[3].d
        L3p = np.sqrt(L3 ** 2 + L4 ** 2)

        A01 = self.serialrobot.transformations[0].dh(q1)
        Pm = np.concatenate((Pm, [1]), axis=0)
        # Express     Pm in the     reference     system     1,    for convenience
        p1 = np.dot(A01.inv().toarray(), Pm.T)
        r = np.linalg.norm(np.array([p1[0], p1[1]]))
        # alpha
        alpha = np.arctan2(-p1[1], p1[0])

        a = (L2 ** 2 + r ** 2 - L3p ** 2) / (2 * r * L2)
        b = (L2 ** 2 + L3p ** 2 - r ** 2) / (2 * L2 * L3p)
        c = (L3 ** 2 + L3p ** 2 - L4 ** 2) / (2 * L3 * L3p)

        if np.abs(a) < 1.0:
            beta = np.arccos(a)
        else:
            print(
                'WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
            beta = np.nan

        if np.abs(b) < 1.0:
            gamma = np.arccos(b)
        else:
            print(
                'WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
            gamma = np.nan

        if np.abs(c) < 1.0:
            eta = np.arccos(c)
        else:
            print(
                'WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
            eta = np.nan

        # elbow  up
        q2_1 = np.pi / 2 - beta - alpha
        # elbow  down
        q2_2 = np.pi / 2 + beta - alpha
        # elbow up
        q3_1 = np.pi - eta - gamma
        # elbow down
        q3_2 = np.pi - eta + gamma
        # joint ranges are considered and we try to restrict the solution in that case.
        if q2_1 < self.joint_ranges[0, 1] or q2_1 > self.joint_ranges[1, 1]:
            q2_1 = np.arctan2(np.sin(q2_1), np.cos(q2_1))
        if q2_2 < self.joint_ranges[0, 1] or q2_2 > self.joint_ranges[1, 1]:
            q2_2 = np.arctan2(np.sin(q2_2), np.cos(q2_2))
        if q3_1 < self.joint_ranges[0, 2] or q3_1 > self.joint_ranges[1, 2]:
            q3_1 = np.arctan2(np.sin(q3_1), np.cos(q3_1))
        if q3_2 < self.joint_ranges[0, 2] or q3_2 > self.joint_ranges[1, 2]:
            q3_2 = np.arctan2(np.sin(q3_2), np.cos(q3_2))
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
        Q = A23.inv() * A12.inv() * A01.inv() * T
        # detect the degenerate case when q(5) = 0, this leads to zeros   % in Q13, Q23, Q31 and Q32 and Q33 = 1
        thresh = 1e-6
        # standard solution
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
            print(50 * '!')
            print('Degenerate')
            # degenerate solution
            q5 = np.real(np.arccos(Q[2, 2]))
            q5_ = q5
            q4 = 0
            q4_ = np.pi
            q6 = np.arctan2(Q[0, 1], -Q[1, 1])
            q6_ = q6 - np.pi
        wrist1 = [q4, q5, q6]
        wrist2 = [q4_, q5_, q6_]
        return np.array(wrist1), np.array(wrist2)


def extend_solutions(qi, qw):
    """
    Adds combinations of +-2pi to the solutions in wrist and concatenates
    """
    q_total = []
    combinations = np.array([[0, 0, 0],
                             [2 * np.pi, 0, 0],
                             [-2 * np.pi, 0, 0],
                             [0, 0, 2 * np.pi],
                             [0, 0, -2 * np.pi],
                             [2 * np.pi, 0, 2 * np.pi],
                             [-2 * np.pi, 0, -2 * np.pi],
                             [2 * np.pi, 0, -2 * np.pi],
                             [-2 * np.pi, 0, 2 * np.pi]])
    for i in range(len(combinations)):
        qwi = qw + np.array(combinations[i])
        qt = np.concatenate((qi, qwi))
        q_total.append(qt)
    return q_total
