#!/usr/bin/env python
# encoding: utf-8
"""
Please open the practicasl/control/twodofrobot.ttt scene before using this class.

The TwoDOFRobot is a simple two DOF planar robot with two rotational joints

@Authors: Arturo Gil
@Time: November 2025
"""
import numpy as np
from artelib.homogeneousmatrix import HomogeneousMatrix
from artelib.parameters import JointParameters, ControlParemeters, DynamicParameters
#from artelib.path_planning import filter_path, path_trapezoidal_i
from artelib.seriallink import SerialRobot
from robots.robot import Robot
import matplotlib.pyplot as plt


class TwoDOFRobot(Robot):
    def __init__(self, simulation):
        robot_DOF = 2
        robot_name = 'TwoDofRobot'
        base_name = '/ROBOTBASE'
        joint_name = 'joint'
        # if moveJ, moveAbsJ are selected as precision=True, then, this time
        # is considered to settle down control and come to a final stop
        settle_time = 0.3
        # joint_rates maximum joint speeds (rad/s), max joint accelerations, joint ranges
        joint_parameters = JointParameters(
            DOF=robot_DOF,
            joint_ranges=np.pi * np.array([[-180.0, -180.0],
                                           [180.0, 180.0]]) / 180.0,
            max_joint_speeds=np.pi * np.array([100.0, 100.0]) / 180.0,
            max_joint_accelerations=np.pi * np.array([500.0, 500.0]) / 180.0,
            settle_time=settle_time)
        # DH parameters of the robot
        serial_parameters = SerialRobot(n=2, T0=np.eye(4), name='TwoDOFRobot')
        serial_parameters.append(th=0, d=0.0, a=0.5, alpha=0, link_type='R')
        serial_parameters.append(th=0, d=0.0, a=0.5, alpha=0, link_type='R')

        # dynamic parameters and simple notation for lengths
        self.m1 = 1.0
        self.m2 = 1.0
        self.masses = [self.m1, self.m2]
        self.g = 9.81
        # lengths
        self.L1 = 0.5
        self.L2 = 0.5
        # self.a = np.array([self.L[0]/2, self.L[1]/2])
        # Caution: the inertia values I1 and I2 must be expressed at
        # the reference frame of the Center Of Mass.
        self.I = [(1 / 12) * self.m1 * self.L1 ** 2, (1 / 12) * self.m2 * self.L2 ** 2]
        dynamic_parameters = DynamicParameters(g=9.81,
                                               masses=self.masses,
                                               inertia=[self.I])
        # define the control parameters and the type of controller
        # uncomment one of the following methods and constants
        # control_type = 'open_loop'
        # kp = np.diag([0.0, 0.0])
        # kd = np.diag([0.0, 0.0])
        # ki = np.diag([0.0, 0.0])
        # control_type = 'PD_precomputed'
        # kp = np.diag([800.0, 800.0])
        # kd = np.diag([20.0, 20.0])
        # ki = np.diag([3.0, 3.0])
        # set this control as default
        control_type = 'acc_compensation'
        kp = np.diag([1200.0, 1200.0])
        kd = np.diag([800.0, 800.0])
        ki = np.diag([50.0, 50.0])
        control_parameters = ControlParemeters(kp=kp, kd=kd, ki=ki, control_type=control_type)
        # init base class attributes
        Robot.__init__(self, DOF=robot_DOF,
                       name=robot_name,
                       base_name=base_name,
                       joint_name=joint_name,
                       simulation=simulation,
                       joint_parameters=joint_parameters,
                       dynamic_parameters=dynamic_parameters,
                       control_parameters=control_parameters,
                       serial_parameters=serial_parameters)

    def inversedynamics(self, q, qd, qdd):
        g = self.g
        I1 = self.I[0]
        I2 = self.I[1]
        m1 = self.m1
        m2 = self.m2
        l1 = self.L1
        l2 = self.L2
        a1 = l1/2
        a2 = l2/2
        # the state in shorthand
        q1 = q[0]
        q2 = q[1]
        qd1 = qd[0]
        qd2 = qd[1]
        qdd1 = qdd[0]
        qdd2 = qdd[1]
        cosq1 = np.cos(q1)
        cosq2 = np.cos(q2)
        # sinq1 = np.sin(q1)
        sinq2 = np.sin(q2)
        cosq1q2 = np.cos(q1+q2)
        # tau = Mqdd+C()+G(q)
        M11 = I1 + I2 + m1 * a1 ** 2 + m2 * (l1 ** 2 + a2 ** 2 + 2 * l1 * a2 * cosq2)
        M12 = I2 + m2*(a2**2 + l1*a2*cosq2)
        M21 = M12
        M22 = I2 + m2*a2**2
        M = np.array([[M11, M12],
                     [M21, M22]])
        # Coriollis
        b = m2*l1*a2
        C1 = -b*sinq2*(2*qd1*qd2+qd2**2)
        C2 = b*sinq2*(qd1**2)
        C = np.array([[C1], [C2]])
        # Gravity
        G1 = (m1*a1 + m2*l1)*g*cosq1 + m2*a2*g*cosq1q2
        G2 = m2*a2*g*cosq1q2
        G = np.array([[G1], [G2]])
        qdd = np.array([qdd1, qdd2])
        tau = np.matmul(M, qdd.T) + C.T + G.T
        return tau[0]