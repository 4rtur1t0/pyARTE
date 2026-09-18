#!/usr/bin/env python
# encoding: utf-8
"""
Please open the practicasl/dynamics/scene/onedofrobot.ttt scene before using this class.

The OneDOFRobot is a simple robot with a single joint

@Authors: Arturo Gil
@Time: April 2024
"""
import numpy as np
from artelib.parameters import JointParameters, DynamicParameters, ControlParemeters
from artelib.seriallink import SerialRobot
from robots.robot import Robot


class OneDOFRobot(Robot):
    def __init__(self, simulation):
        robot_DOF = 1
        robot_name = 'OneDofRobot'
        base_name = '/ROBOTBASE'
        joint_name = 'joint'
        # if moveJ, moveAbsJ are selected as precision=True, then, this time
        # is considered to settle down control and come to a final stop
        settle_time = 0.2
        # joint_rates maximum joint speeds (rad/s), max joint accelerations, joint ranges
        joint_parameters = JointParameters(
            DOF=robot_DOF,
            joint_ranges=np.pi * np.array([[-180.0],
                                           [180.0]]) / 180.0,
            max_joint_speeds=np.pi * np.array([100.0]) / 180.0,
            max_joint_accelerations=np.pi * np.array([500.0]) / 180.0,
            settle_time=settle_time)
        # DH parameters of the robot
        serial_parameters = SerialRobot(n=1, T0=np.eye(4), name='OneDOFRobot')
        serial_parameters.append(th=0, d=0.352, a=0.07, alpha=-np.pi / 2, link_type='R')

        masses = [1.0]
        self.L = 1.0
        self.I = (1.0 / 3.0) * masses[0] * self.L ** 2
        dynamic_parameters = DynamicParameters(g=9.81,
                                               masses=masses,
                                               inertia=[self.I])
        # define the control parameters and the type of controller
        # control_type = 'PD_precomputed'
        # kp = np.diag([1550.0])
        # kd = np.diag([550.0])
        # ki = np.diag([50.0])
        control_type = 'acc_compensation'
        kp = np.diag([1550.0])
        kd = np.diag([550.0])
        ki = np.diag([50.0])
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
        I = self.dynamic_parameters.inertia[0]
        m = self.dynamic_parameters.masses[0]
        g = self.dynamic_parameters.g
        L = self.L
        # q, qd, qdd = self.get_state()
        tau = I*qdd + m*g*(L/2)*np.cos(q)
        return tau