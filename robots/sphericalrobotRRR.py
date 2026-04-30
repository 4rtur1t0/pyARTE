#!/usr/bin/env python
# encoding: utf-8
"""
Please open the practicasl/control/twodofrobot.ttt scene before using this class.

The TwoDOFRobot is a simple two DOF planar robot with two rotational joints

@Authors: Arturo Gil
@Time: November 2025
"""
import numpy as np
from artelib.parameters import JointParameters, DynamicParameters, ControlParemeters
from artelib.seriallink import SerialRobot
from robots.robot import Robot
import matplotlib.pyplot as plt
from dynamics.sphericalRRR.func_M import func_M
from dynamics.sphericalRRR.func_G import func_G
from dynamics.sphericalRRR.func_C import func_C


class SphericalRobotRRR(Robot):
    def __init__(self, simulation):
        robot_DOF = 3
        robot_name = 'SphericalRRR'
        base_name = '/ROBOTBASE'
        joint_name = 'joint'
        settle_time = 0.3
        # joint_rates maximum joint speeds (rad/s), max joint accelerations, joint ranges
        joint_parameters = JointParameters(joint_ranges=np.pi * np.array([[-180, -180, -180],
                                                                          [180, 180, 180]]) / 180.0,
                                           max_joint_speeds=np.pi * np.array([100, 100, 100]) / 180.0,
                                           max_joint_accelerations=np.pi * np.array([500, 500, 500]) / 180.0,
                                           settle_time=settle_time,
                                           max_linear_velocity=1.0,
                                           max_linear_acceleration=10.0,
                                           max_angular_velocity=2.0,
                                           max_angular_acceleration=20.0,
                                           DOF=robot_DOF)

        serial_parameters = SerialRobot(n=3, T0=np.eye(4), name='SphericalRobotRRR')
        serial_parameters.append(th=0, d=0.3, a=0.0, alpha=np.pi / 2, link_type='R')
        serial_parameters.append(th=0, d=0.0, a=0.5, alpha=0, link_type='R')
        serial_parameters.append(th=0, d=0.0, a=0.5, alpha=0, link_type='R')

        # dynamic parameters and simple notation for lengths
        masses = [10.0, 10.0, 10.0]
        inertia = np.array([[.1, .1, .1],
                            [.1, .1, .1],
                            [.1, .1, .1]])
        L1 = 0.2
        L2 = 0.5
        L3 = 0.5
        dynamic_parameters = DynamicParameters(g=9.81,
                                               masses=masses,
                                               inertia=inertia,
                                               lengths=[L1, L2, L3])

        # define the control parameters and the type of controller
        control_type = 'acc_compensation'
        kp = np.diag([2000.0, 2000.0, 2000.0])
        kd = np.diag([100.0, 100.0, 100.0])
        ki = np.diag([0.0, 0.0, 0.0])
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

    def inverse_kinematics(self, target_position, target_orientation, extended=False):
        return

    def inverse_dynamics(self, q, qd, qdd):
        """
        The symbolic dynamics model of 
        """
        g = self.dynamic_parameters.g
        I1x = self.dynamic_parameters.inertia[0, 0]
        I1y = self.dynamic_parameters.inertia[0, 1]
        I1z = self.dynamic_parameters.inertia[0, 2]
        I2x = self.dynamic_parameters.inertia[1, 0]
        I2y = self.dynamic_parameters.inertia[1, 1]
        I2z = self.dynamic_parameters.inertia[1, 2]
        I3x = self.dynamic_parameters.inertia[2, 0]
        I3y = self.dynamic_parameters.inertia[2, 1]
        I3z = self.dynamic_parameters.inertia[2, 2]
        m1 = self.dynamic_parameters.masses[0]
        m2 = self.dynamic_parameters.masses[1]
        m3 = self.dynamic_parameters.masses[2]
        L1 = self.dynamic_parameters.lengths[0] #L1
        L2 = self.dynamic_parameters.lengths[1] #L2
        L3 = self.dynamic_parameters.lengths[2] #L3
        # the state in shorthand
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        qd1 = qd[0]
        qd2 = qd[1]
        qd3 = qd[2]
        qdd1 = qdd[0]
        qdd2 = qdd[1]
        qdd3 = qdd[2]

        M = func_M(I2x,I3x,I2y,I3y,I1z,I2z,I3z,L2,L3,m2,m3,q2,q3)
        C = func_C(I2x,I3x,I2z,I3z,L2,L3,m2,m3,q2,q3,qd1,qd2,qd3)
        G = func_G(L2,L3,g,m2,m3,q2,q3)

        qdd = np.array([qdd1, qdd2, qdd3])
        tau = np.matmul(M, qdd.T) + C.T + G.T
        return tau[0]




