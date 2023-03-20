#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/ur5.ttt scene before using this class.

RobotUR5 is a derived class of the Robot base class

@Authors: Arturo Gil
@Time: April 2021
"""
import sim
import numpy as np
from artelib import homogeneousmatrix
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.inverse_kinematics import delta_q
from artelib.seriallink import SerialRobot
from artelib.tools import compute_kinematic_errors
from robots.robot import Robot
from kinematics.kinematics_ur5 import eval_symbolic_jacobian_UR5


class RobotUR5(Robot):
    def __init__(self, clientID):
        # init base class attributes
        Robot.__init__(self)
        self.clientID = clientID
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
        self.max_error_dist_inversekinematics = 0.01
        self.max_error_orient_inversekinematics = 0.01
        # self.ikmethod = 'transpose'
        self.ikmethod = 'moore-penrose-damped'
        # self.ikmethod = 'moore-penrose'
        # whether to apply joint limits in inversekinematics
        self.do_apply_joint_limits = True
        self.epsilonq = 0.005

        self.serialrobot = SerialRobot(n=6, T0=np.eye(4), name='UR5')
        self.serialrobot.append(th=-np.pi/2, d=0.089159, a=0, alpha=np.pi/2)
        self.serialrobot.append(th=+np.pi/2, d=0,        a=0.425, alpha=0)
        self.serialrobot.append(th=0,        d=0,        a=0.39225, alpha=0)
        self.serialrobot.append(th=-np.pi/2, d=0.10915,  a=0, alpha=-np.pi/2)
        self.serialrobot.append(th=0,        d=0.09465,  a=0, alpha=np.pi/2)
        self.serialrobot.append(th=0,        d=0.0823,   a=0, alpha=0)

    def start(self, base_name='/UR5', joint_name='UR5_joint'):
        errorCode, robotbase = sim.simxGetObjectHandle(self.clientID, base_name, sim.simx_opmode_oneshot_wait)
        errorCode, q1 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '1',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q2 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '2',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q3 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '3',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q4 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '4',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q5 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '5',
                                                sim.simx_opmode_oneshot_wait)
        errorCode, q6 = sim.simxGetObjectHandle(self.clientID, base_name + '/' + joint_name + '6',
                                                sim.simx_opmode_oneshot_wait)

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

    def inversekinematics(self, q0, target_position, target_orientation, extended=None):
        """
        Solve the inverse kinematics using a Jacobian method.
        target_position: XYX vector in global coordinates.
        target_orientation: A quaternion specifying orientation.
        """
        # build transform using position and Quaternion
        Ttarget = HomogeneousMatrix(target_position, target_orientation)
        q = q0
        for i in range(0, self.max_iterations_inverse_kinematics):
            print('Iteration number: ', i)
            Ti = self.directkinematics(q)
            e, error_dist, error_orient = compute_kinematic_errors(Tcurrent=Ti, Ttarget=Ttarget)
            print('errordist, error orient: ', error_dist, error_orient)
            if error_dist < self.max_error_dist_inversekinematics and error_orient < self.max_error_orient_inversekinematics:
                print('Converged!!')
                break
            J, Jv, Jw = self.manipulator_jacobian(q)
            qd = delta_q(J, e, method=self.ikmethod)
            q = q + qd
            if self.do_apply_joint_limits:
                [q, _] = self.apply_joint_limits(q)
        return q

    def moveAbsJ(self, q_target, precision=True):
        """
        Commands the robot to the specified joint target positions.
        The targets are filtered and the robot is not commanded whenever a single joint is out of range.
        """
        # remove joints out of range and get the closest joint
        total, partial = self.check_joints(q_target)
        if total:
            self.set_joint_target_positions(q_target, precision=precision)
        else:
            print('moveABSJ ERROR: joints out of range')

    def moveJ(self, target_position, target_orientation, precision=True):
        """
        Commands the robot to a target position and orientation.
        All solutions to the inverse kinematic problem are computed. The closest solution to the
        current position of the robot q0 is used
        """
        q0 = self.q_current
        # resultado filtrado. Debe ser una matriz 6xn_movements
        # CAUTION. This calls the inverse kinematic method of the derived class
        qs = self.inversekinematics(q0=q0, target_position=target_position,
                                    target_orientation=target_orientation)
        # remove joints out of range and get the closest joint
        # 7 rows, 1 column of solutions
        qs = self.filter_joint_limits(np.array([qs]).T)

        # comandar al robot hasta
        self.set_joint_target_positions(qs, precision=precision)

    def moveL(self, target_position, target_orientation, precision=False, vmax=0.8, wmax=0.2):
        q0 = self.q_current
        # resultado filtrado. Debe ser una matriz 6xn_movements
        qs = self.inversekinematics_line(q0=q0, target_position=target_position,
                                         target_orientation=target_orientation,
                                         vmax=vmax, wmax=wmax)
        # command the robot
        self.set_joint_target_positions(qs, precision=precision)