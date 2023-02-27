#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before using this class.

RobotABBIRB140 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib import homogeneousmatrix
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.seriallink import SerialRobot
from artelib.tools import buildT, normalize_angle
from robots.robot import Robot
import sim
from artelib.path_planning import generate_target_positions, generate_target_orientations_Q, \
    move_target_positions_obstacles, n_movements

class RobotABBIRB140(Robot):

    def __init__(self, clientID):
        # init base class attributes
        Robot.__init__(self)
        self.clientID = clientID

        # maximum joint speeds (rad/s)
        max_joint_speeds = np.array([180, 180, 180, 180, 180, 180, 180])
        self.max_joint_speeds = max_joint_speeds * np.pi / 180.0
        # max and min joint ranges (an
        joint_ranges = np.array([[-180, -90, -230, -200, -115, -400],
                                 [180,   110,  50,  200,  115, 400]])
        self.joint_ranges = joint_ranges * np.pi / 180.0

        self.max_iterations_inverse_kinematics = 15000
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        self.ikmethod = 'closed-equations'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True
        self.epsilonq = 0.0001

        # DH parameters of the robot
        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), name='ABBIRB140')
        self.serialrobot.append(th=0, d=0.352, a=0.07, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=-np.pi/2, d=0, a=0.36, alpha=0, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0.38, a=0, alpha=np.pi / 2, link_type='R')
        self.serialrobot.append(th=0, d=0, a=0, alpha=-np.pi / 2, link_type='R')
        self.serialrobot.append(th=np.pi, d=0.065, a=0, alpha=0, link_type='R')

    def start(self, base_name='/IRB140', joint_name='joint'):
        armjoints = []
        # Get the handles of the relevant objects
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, base_name, sim.simx_opmode_oneshot_wait)

        errorCode, q1 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '1', sim.simx_opmode_oneshot_wait)
        errorCode, q2 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '2', sim.simx_opmode_oneshot_wait)
        errorCode, q3 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '3', sim.simx_opmode_oneshot_wait)
        errorCode, q4 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '4', sim.simx_opmode_oneshot_wait)
        errorCode, q5 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '5', sim.simx_opmode_oneshot_wait)
        errorCode, q6 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '6', sim.simx_opmode_oneshot_wait)

        armjoints.append(q1)
        armjoints.append(q2)
        armjoints.append(q3)
        armjoints.append(q4)
        armjoints.append(q5)
        armjoints.append(q6)
        # must store the joints
        self.joints = armjoints

    def inversekinematics(self, target_position, target_orientation, q0=None):
        """
        Inverse kinematic method for the ABB IRB140 robot.
        Please, beware that the ABB robot corresponds to a modified version of the original robot that is included in
        Coppelia. In particular, the movement direction of joint2 and joint3 have been reversed and now match the
        positive direction specified by the manufacturer.
        """
        # Ttarget = buildT(target_position, target_orientation)
        Ttarget = HomogeneousMatrix(target_position, target_orientation)

        # Remove Ttcp, so that T_end_effector is specified
        Tcp_inv = self.Ttcp.inv()
        Ttarget = Ttarget*Tcp_inv

        # get the value from the robot class (last link length)
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
        # make them real numbers!
        q = np.real(q)

        # normalize q1 (first row)
        q[0, :] = normalize_angle(q[0, :])

        final_q_solution = []
        # solve the last three joints
        # skip nan values
        for i in range(0, 8, 2):
            qi = q[0:6, i]
            qip = q[0:6, i+1]
            # if q1 or q2 or q3 are invalid (nan), then continue
            if np.isnan(np.sum(qi)):
                continue
            w1, w2 = self.solve_spherical_wrist(qi, Ttarget)
            # append the two solutions to the last three joints
            qi[3:6] = normalize_angle(w1)
            final_q_solution.append(qi)
            qip[3:6] = normalize_angle(w2)
            final_q_solution.append(qip)
        final_q_solution = np.array(final_q_solution)
        # transpose, solutions are arranged by columns
        final_q_solution = final_q_solution.T
        return final_q_solution

    def solve_for_theta23(self, q1, Pm):
        # See arm geometry
        L2 = self.serialrobot.transformations[1].a
        L3 = self.serialrobot.transformations[3].d
        A01 = self.serialrobot.transformations[0].dh(q1)
        Pm = np.concatenate((Pm, [1]), axis=0)
        # Express     Pm in the     reference     system     1,    for convenience
        p1 = np.dot(A01.inv().toarray(), Pm.T)
        r = np.linalg.norm(np.array([p1[0], p1[1]]))
        beta = np.arctan2(-p1[1], p1[0])

        a = (L2**2 + r**2 - L3**2) / (2 * r *L2)
        b = (L2**2 + L3**2 - r**2) / (2 * L2 * L3)

        if np.abs(a) < 1.0:
            gamma = np.arccos(a)
        else:
            print('WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
            gamma = np.nan

        if np.abs(b) < 1.0:
            eta = np.arccos(b)
        else:
            print('WARNING: ONE OF THE INVERSE KINEMATIC SOLUTIONS IS NOT FEASIBLE (ABB IRB140 ROBOT). The point is out of the workspace')
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


    def inversekinematics_line(self, target_position, target_orientation, vmax=1.0, q0=None):
        """
        The end effector should follow a line in task space to reach target position and target orientation.
        A number of points is interpolated along the line, according to the speed vmax and simulation time
        (delta_time).
        The same number or points are also interpolated in orientation.
        Caution. target_orientationQ is specified as a quaternion
        """
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        Ti = self.directkinematics(q0)
        Qcurrent = Ti.Q()
        Qtarget = target_orientation.Q()
        p_current = Ti.pos()
        p_target = Ttarget.pos()
        n = n_movements(p_current, p_target, vmax)
        # generate n target positions
        target_positions = generate_target_positions(p_current, p_target, n)
        # generating quaternions on the line. Use SLERP to interpolate between quaternions
        target_orientations = generate_target_orientations_Q(Qcurrent, Qtarget, len(target_positions))

        q_path = []
        q = q0

        # now try to reach each target position on the line
        for i in range(len(target_positions)):
            q = self.inversekinematics(target_position=target_positions[i],
                                       target_orientation=target_orientations[i], q0=q)
            q_path.append(q)
        return q_path
