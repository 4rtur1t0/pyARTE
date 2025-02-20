#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

RobotUR5 is a derived class of the Robot base class

@Authors: Arturo Gil
@Time: April 2021
@Revision: July 2023, Arturo Gil
"""
import numpy as np
from artelib import homogeneousmatrix
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import delta_q
from artelib.seriallink import SerialRobot
from artelib.tools import compute_kinematic_errors
from robots.robot import Robot
from kinematics.kinematics_ur5 import eval_symbolic_jacobian_UR5
from artelib.tools import normalize_angle


class RobotUR5(Robot):
    def __init__(self, simulation):
        # init base class attributes
        Robot.__init__(self, simulation=simulation)

        self.DOF = 6
        self.q_current = np.zeros((1, self.DOF))
        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges
        joint_ranges = np.array([[-360, -360, -360, -360, -360, -360],
                                 [360,   360,  360,  360,  360,  360]])
        self.joint_ranges = joint_ranges * np.pi / 180.0
        self.max_iterations_inverse_kinematics = 1500
        self.max_error_dist_inversekinematics = 0.002
        self.max_error_orient_inversekinematics = 0.005
        # self.ikmethod = 'transpose'
        self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True
        self.epsilonq = 0.005

        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), name='UR5')
        self.serialrobot.append(th=0, d=0.0892, a=0, alpha=np.pi / 2, link_type='R')
        self.serialrobot.append(th=np.pi / 2, d=0, a=0.4251, alpha=0, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0.39215, alpha=0, link_type='R')
        self.serialrobot.append(th=-np.pi / 2, d=0.11, a=0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0.09465, a=0, alpha=np.pi / 2, link_type='R')
        self.serialrobot.append(th=-np.pi / 2, d=0.08295, a=0, alpha=0, link_type='R')


    def start(self, base_name='/UR5', joint_name='joint'):
        robotbase = self.simulation.sim.getObject(base_name)
        q1 = self.simulation.sim.getObject(base_name + '/' + joint_name + '1')
        q2 = self.simulation.sim.getObject(base_name + '/' + joint_name + '2')
        q3 = self.simulation.sim.getObject(base_name + '/' + joint_name + '3')
        q4 = self.simulation.sim.getObject(base_name + '/' + joint_name + '4')
        q5 = self.simulation.sim.getObject(base_name + '/' + joint_name + '5')
        q6 = self.simulation.sim.getObject(base_name + '/' + joint_name + '6')

        joints = []
        joints.append(q1)
        joints.append(q2)
        joints.append(q3)
        joints.append(q4)
        joints.append(q5)
        joints.append(q6)
        self.joints = joints

    def get_symbolic_jacobian(self, q):
        J, Jv, Jw = eval_symbolic_jacobian_UR5(q)
        return J, Jv, Jw

    # def inversekinematics(self, q0, target_position, target_orientation, extended=None):
    #     """
    #     Solve the inverse kinematics using a Jacobian method.
    #     target_position: XYX vector in global coordinates.
    #     target_orientation: A quaternion specifying orientation.
    #     """
    #     # build transform using position and Quaternion
    #     Ttarget = HomogeneousMatrix(target_position, target_orientation)
    #     q = q0
    #     for i in range(0, self.max_iterations_inverse_kinematics):
    #         print('Iteration number: ', i)
    #         Ti = self.directkinematics(q)
    #         e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
    #         print('errordist, error orient: ', error_dist, error_orient)
    #         if error_dist < self.max_error_dist_inversekinematics and error_orient < self.max_error_orient_inversekinematics:
    #             print('Converged!!')
    #             break
    #         J, Jv, Jw = self.manipulator_jacobian(q)
    #         qd = delta_q(J, e, method=self.ikmethod)
    #         q = q + qd
    #         if self.do_apply_joint_limits:
    #             [q, _] = self.apply_joint_limits(q)
    #     return q

    def inversekinematics(self, target_position, target_orientation, q0=None, extended=True):
        """
        Solve the inverse kinematics using a closed method.
        The technique used here is based on a geometrical analysis of the robot.
        target_position: XYX vector in global coordinates.
        target_orientation: A quaternion specifying orientation.
        """
        # build transform using position and Quaternion
        T = HomogeneousMatrix(target_position, target_orientation)
        # action: compute q1 / so that the robot may be  at any  of the possible  pms.
        [q1A, q1B] = self.solve_for_q1(T)

        # two possible  solutions for q2 and q3 given the two possible pm0 and pm1
        [q2Awu, q3Awu, z4Awu] = self.solve_for_q2_q3(T, q1A, +1)
        [q2Awd, q3Awd, z4Awd] = self.solve_for_q2_q3(T, q1A, -1)
        [q2Bwu, q3Bwu, z4Bwu] = self.solve_for_q2_q3(T, q1B, +1)
        [q2Bwd, q3Bwd, z4Bwd] = self.solve_for_q2_q3(T, q1B, -1)

        # now, group solutions
        # group 1: q1A(first  solution)
        #          z4 up, with  elbow  up(1) and down(2)
        qgroup1 = np.array([[q1A,          q1A],
                            [q2Awu[0],     q2Awu[1]],
                            [q3Awu[0],     q3Awu[1]],
                            [0,            0],
                            [0,            0],
                            [0,            0]])
        # find the  corresponding angles for q4, q5, q6
        for i in range(2):
            qi = qgroup1[:, i]
            [q4, q5, q6] = self.solve_for_UR_wrist(T=T, q=qi, z4=z4Awu)
            qgroup1[3, i] = q4
            qgroup1[4, i] = q5
            qgroup1[5, i] = q6
        # group 2: q1A (first solution)
        #          z4 down, elbow up(1) and down(2)
        qgroup2 = np.array([[q1A,       q1A],
                            [q2Awd[0],  q2Awd[1]],
                            [q3Awd[0],  q3Awd[1]],
                            [0,         0],
                            [0,         0],
                            [0,         0]])
        # find the corresponding angles for q4, q5, q6
        for i in range(2):
            qi = qgroup2[:, i]
            [q4, q5, q6] = self.solve_for_UR_wrist(T=T, q=qi, z4=z4Awd)
            qgroup2[3, i] = q4
            qgroup2[4, i] = q5
            qgroup2[5, i] = q6

        # group 3: q1B (second solution for q1)
        #          z4 up, elbow up(1) and down(2)
        qgroup3 = np.array([[q1B,      q1B],
                            [q2Bwu[0], q2Bwu[1]],
                            [q3Bwu[0], q3Bwu[1]],
                            [0,        0],
                            [0,        0],
                            [0,        0]])
        # find the corresponding angles for q4, q5, q6
        for i in range(2):
            qi = qgroup3[:, i]
            [q4, q5, q6] = self.solve_for_UR_wrist(T=T, q=qi, z4=z4Bwu)
            qgroup3[3, i] = q4
            qgroup3[4, i] = q5
            qgroup3[5, i] = q6

        # group 4: q1B (second solution for q1)
        #          z4 down, elbow up(1) and down(2)
        qgroup4 = np.array([[q1B,        q1B],
                           [q2Bwd[0],    q2Bwd[1]],
                           [q3Bwd[0],    q3Bwd[1]],
                           [0,           0],
                           [0,           0],
                           [0,           0]])
        # find the corresponding angles for q4, q5, q6
        for i in range(2):
            qi = qgroup4[:, i]
            [q4, q5, q6] = self.solve_for_UR_wrist(T=T, q=qi, z4=z4Bwd)
            qgroup4[3, i] = q4
            qgroup4[4, i] = q5
            qgroup4[5, i] = q6
        # arrange all solutions together
        # q = np.array([qgroup1, qgroup2, qgroup3, qgroup4])
        q = np.array(qgroup1)
        q = np.append(q, qgroup2, axis=1)
        q = np.append(q, qgroup3, axis=1)
        q = np.append(q, qgroup4, axis=1)
        # now, filter out possible imaginary solutions or Nones
        q_filtered = np.array([])
        # filter all solutions... if any solution is complex remove the whole column
        # complex solutions are marked with nan in the solve_for_q2_q3 function.
        valid = 0
        for i in range(q.shape[1]):
            qi = q[:, i]
            n = np.isnan(qi)
            # if found any nan in the solution, skip this solution
            if any(n):
                continue
            if valid == 0:
                q_filtered = np.array(qi)
                valid += 1
            else:
                q_filtered = np.vstack((q_filtered, qi))

        q = q_filtered.T
        return q

    def solve_for_q1(self, T):
        """
        Solve for q1, given the point pw
        """
        # See geometry at the reference for this robot. Get parameters from the DH table
        L6 = self.serialrobot.transformations[5].d
        L4 = self.serialrobot.transformations[3].d
        p = T[0:3, 3]
        # z5 is parallel to z6 (aligned)
        z5 = T[0:3, 2]

        # Pw: wrist position(false wrist or pseudo wrist)
        # however all  possible pw lie on a circle around z3
        # Compute pw that lies in the intersection of z5 with z4
        pw = p - L6 * z5
        R = np.sqrt(pw[0]**2 + pw[1]**2)
        # caution, clip to 0, 1, to avoid error near singular points
        alpha = np.arcsin(np.clip(L4 / R, 0.0, 1.0))
        beta = np.arctan2(pw[1], pw[0])
        # Two differen solutions exsit for q1. Beware of the sign in alpha
        q1A = alpha + beta
        q1B = np.pi - alpha + beta
        # normalize to [-pi, pi]
        q1B = np.arctan2(np.sin(q1B), np.cos(q1B))
        return q1A, q1B

    def solve_for_q2_q3(self, T, q1, signo):
        """
        Given q1, solve for q2 and q3. A solution is obtained for +z4.And a different one for -z4.
        Caution, the solutions for
        """
        # See geometry at the reference for this robot. Get parameters from the DH table
        L2 = self.serialrobot.transformations[1].a
        L3 = self.serialrobot.transformations[2].a
        L4 = self.serialrobot.transformations[3].d
        L5 = self.serialrobot.transformations[4].d
        L6 = self.serialrobot.transformations[5].d
        p = T[0:3, 3]
        # z5 is parallel to z6(aligned)
        z5 = T[0:3, 2]

        # action 2: compute z1 parallel to z2 and parallel to z3
        # given that q1 is known, compute z1, and assign to z3 since they are parallel axes
        A01 = self.serialrobot.dh(np.array([q1, 0, 0, 0, 0, 0]), 0)
        z3 = A01[0:3, 2]

        # Now compute + -z4, given that z3 and z5 are both known since z3 is perpendicular to z4 and z5 is also
        # perpendicular to z4
        # % if norm z4a < threshold, then z5 = z3
        # two solutions + -
        z4 = np.cross(z3, z5)
        if np.linalg.norm(z4) > 0.01:
            # compute two different solutions depending on the direction of z4
            # this is the standard solution with z4 perpendicular to the plane formed by z3 and z5
            z4 = signo * z4 / np.linalg.norm(z4)
        else:
            # z3 and z5 are parallel
            print('CAUTION, SINGULAR CONDITION DETECTED: z3 and z5 are ALIGNED')
            print('Z4 NORM IS: ', np.linalg.norm(z4))  # % if norm(z4) == 0 --> z singular condition
            # whenever z3 and z5 are parallel, we are at a singular point with a null
            # det(J) and infinite solutions. In this case, we may use the vector
            # z4 = [0,0,1], which is perpendicular to z3 and, in this case also to z5
            z4 = signo*np.array([0, 0, 1])
            # x3 = A01[0:3, 0]
            # z4 = x3

        # Now, given that z3, z4 and z5 are known, compute pm
        # pm is computed for this solution particular configuration of z4
        # pm is the origin of the X3Y3Z3 DH reference system
        pm0 = p - L6 * z5 - L5 * z4 - L4 * z3
        pm0 = np.append(pm0, [1])
        pm1 = np.dot(A01.inv().array, pm0.T)
        # the Euclidian distance from the origin of X1Y1Z1 to the origin of X3Y3Z3
        R = np.sqrt(pm1[0]**2 + pm1[1]**2)
        if R > L2+L3:
            print('CAUTION: The point is not reachable by the robot')
            q2 = np.array([np.nan, np.nan])
            q3 = np.array([np.nan, np.nan])
            return q2, q3, z4
        alpha = np.arctan2(pm1[1], pm1[0])
        # caution, clipping to -1, 1, since the point pm1 should be in range now
        cbeta = np.clip((L2 ** 2 + R ** 2 - L3 ** 2) / (2 * L2 * R), -1.0, 1.0)
        ceta = np.clip((L2 ** 2 + L3 ** 2 - R ** 2) / (2 * L2 * L3), -1.0, 1.0)
        beta = np.arccos(cbeta)
        eta = np.arccos(ceta)
        # two possible soutions for q2
        q2up = alpha + beta - np.pi / 2
        q2down = alpha - beta - np.pi / 2
        # two possible solutions for q3
        q3up = eta - np.pi
        q3down = np.pi - eta
        q2 = np.array([q2up, q2down])
        q3 = np.array([q3up, q3down])
        q2 = normalize_angle(q2)
        q3 = normalize_angle(q3)
        return q2, q3, z4

    def solve_for_UR_wrist(self, T, q, z4):
        """
        Given that q1, q2 and q3 are known, compute q4, q5 and q6
        z4 is passed, since the solutions for q1, q2 and q3 depend upon z4 (in a pseudo wrist up/down
        """
        z6 = T[0:3, 2]
        z5 = z6
        x6 = T[0:3, 0]

        # compute q4, since z4 is known and also x3 and y3
        A01 = self.serialrobot.dh(q, 0)
        A12 = self.serialrobot.dh(q, 1)
        A23 = self.serialrobot.dh(q, 2)
        A03 = A01 * A12 * A23
        x3 = A03[0:3, 0]
        y3 = A03[0:3, 1]
        a = np.dot(z4, x3)
        b = np.dot(z4, y3)
        q4 = np.arctan2(b, a)

        # compute q5, since z5 is known and also x4 and y4
        A34 = self.serialrobot.dh(np.array([0, 0, 0, q4, 0, 0]), 3)
        A04 = A03 * A34
        x4 = A04[0:3, 0]
        y4 = A04[0:3, 1]
        a = np.dot(z5, -y4)
        b = np.dot(z5, x4)
        q5 = np.arctan2(b, a)

        # compute q6, since q5 is known and also x6 (from T)
        A45 = self.serialrobot.dh(np.array([0, 0, 0, 0, q5, 0]), 4)
        A05 = A04 * A45
        x5 = A05[0:3, 0]
        y5 = A05[0:3, 1]
        a = np.dot(x6, -y5)
        b = np.dot(x6, x5)
        q6 = np.arctan2(b, a)
        return q4, q5, q6

