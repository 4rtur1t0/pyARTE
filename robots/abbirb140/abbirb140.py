#!/usr/bin/env python
# encoding: utf-8
"""
Please open the scenes/irb140.ttt scene before using this class.

RobotABBIRB140 is a derived class of the Robot base class that

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np
from artelib.parameters import JointParameters, DynamicParameters, ControlParemeters, CoppeliaJoints
from artelib.seriallink import SerialRobot
from dynamics.abbirb140.inverse_dynamics_abbirb140 import inverse_dynamics_abbirb140
from kinematics.abbirb140.inverse_kinematics_abbirb140 import inverse_kinematics_abbirb140, check_can_be_reached_abbirb140
from robots.robot import Robot


class RobotABBIRB140(Robot):
    def __init__(self, simulation, frame=None):
        robot_DOF = 6
        robot_name = 'ABBIRB140'
        base_name = '/IRB140'
        joint_name = 'joint'
        settle_time = 0.05
        # joint_rates maximum joint speeds (rad/s), max joint accelerations, joint ranges
        # also stores max linear speed and max linear acceleration for
        joint_parameters = JointParameters(joint_ranges=np.pi*np.array([[-180.0, -90.0, -230.0, -400.0, -115.0, -400.0],
                                                                        [180.0,   110.0,  50.0,  400.0,  115.0, 400.0]])/180.0,
                                           max_joint_speeds=np.pi*np.array([200.0, 200.0, 245.0, 348.0, 360.0, 450.0])/180.0,
                                           max_joint_accelerations=np.pi*np.array([2000.0, 2000.0, 2450.0, 3480.0, 3600.0, 4500.0])/180.0,
                                           settle_time=settle_time,
                                           max_linear_velocity=4.0,         # m/s
                                           max_linear_acceleration=20.0,    # m/s/s
                                           max_angular_velocity=3.0,        # rad/s
                                           max_angular_acceleration=20.0,   # rad/s/s
                                           DOF=robot_DOF)
        # DH parameters of the robot
        serial_parameters = SerialRobot(n=robot_DOF, T0=np.eye(4), name=robot_name)
        serial_parameters.append(th=0.0, d=0.352, a=0.07, alpha=-np.pi / 2, link_type='R')
        serial_parameters.append(th=-np.pi / 2, d=0.0, a=0.36, alpha=0.0, link_type='R')
        serial_parameters.append(th=0.0, d=0.0, a=0.0, alpha=-np.pi / 2, link_type='R')
        serial_parameters.append(th=0.0, d=0.38, a=0.0, alpha=np.pi / 2, link_type='R')
        serial_parameters.append(th=0.0, d=0.0, a=0.0, alpha=-np.pi / 2, link_type='R')
        serial_parameters.append(th=np.pi, d=0.065, a=0.0, alpha=0.0, link_type='R')

        # Dynamic parameters. masses: kg, inertia: kg*m²
        masses = [30.0, 30.0, 30.0, 20.0, 10.0, 5.0]
        inertia = np.array([[3.0, 3.0, 3.0],
                            [3.0, 3.0, 3.0],
                            [3.0, 3.0, 3.0],
                            [2.0, 2.0, 2.0],
                            [1.0, 1.0, 1.0],
                            [0.25, 0.25, 0.25]])
        dynamic_parameters = DynamicParameters(g=9.81,
                                               masses=masses,
                                               inertia=inertia)
        # define the control parameters and the type of controller
        # control_type = 'PD_precomputed'
        # kp = np.diag([150.0, 350.0, 350.0, 150.0, 150.0, 50.0])
        # kd = np.diag([10.0, 10.0, 10.0, 10.0, 10.0, 2.0])
        # ki = np.diag([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        control_type = 'acc_compensation'
        kp = np.diag([4000.0, 4000.0, 4000.0, 4000.0, 4000.0, 4000.0])
        kd = np.diag([200.0, 300.0, 300.0, 300.0, 300.0, 100.0])
        ki = np.diag([200.0, 200.0, 200.0, 200.0, 200.0, 20.0])
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
                       serial_parameters=serial_parameters,
                       frame=frame)

    def inverse_kinematics(self, target_position, target_orientation, extended=False, q0=None):
        """
        Inverse kinematic method for the ABB IRB140 robot.

        Please, beware that the ABB robot corresponds to a modified version of the original robot that is included in
        Coppelia. In particular, the movement direction of joint2 and joint3 have been reversed and now match the
        positive direction specified by the manufacturer (ABB).

        Generally, given an end effector position and orientation, 8 different solutions are provided for the inverse
        kinematic problem. If the extended option is enabled, some extra solutions are provided. These solutions exist,
        given that the joint ranges for q4 and q6 are [-400, 400] degrees.
        """
        qinv = inverse_kinematics_abbirb140(self, target_position, target_orientation, extended=extended, q0=q0)
        return qinv

    def inverse_dynamics(self, q, qd, qdd):
        tau = inverse_dynamics_abbirb140(self, q, qd, qdd)
        return tau

    def check_can_be_reached(self, target_positions, target_orientations):
        """
        Check that the target positions and orientations can be reached by the robot.
        This is a specific function to each robot, since normally involves solving the
        inversekinematic for each target and observing if real solutions exist.
        """
        range_total, range_partial = check_can_be_reached_abbirb140(self, target_positions, target_orientations)
        return range_total, range_partial

