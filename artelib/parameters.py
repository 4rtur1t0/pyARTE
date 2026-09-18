#!/usr/bin/env python
# encoding: utf-8
"""
Classes to store robot parameters (joint ranges, max joint speeds...)

@Authors: Arturo Gil
@Time: Febrer de 2026
"""
import numpy as np
import matplotlib.pyplot as plt


class JointParameters():
    """
        Store the joint parameters. Joint ranges, max_joint_speeds, max_joint_accelerations
    """
    def __init__(self, DOF, joint_ranges, max_joint_speeds, max_joint_accelerations,
                 max_linear_velocity,
                 max_linear_acceleration,
                 max_angular_velocity,
                 max_angular_acceleration,
                 settle_time):
        self.DOF = DOF
        self.joint_ranges = joint_ranges
        self.max_joint_speeds = max_joint_speeds
        self.max_joint_accelerations = max_joint_accelerations
        self.max_linear_velocity = max_linear_velocity
        self.max_linear_acceleration = max_linear_acceleration
        self.max_angular_velocity = max_angular_velocity
        self.max_angular_acceleration = max_angular_acceleration
        self.settle_time = settle_time


class DynamicParameters():
    def __init__(self, masses, g, inertia, r_com=None, euler_com=None, lengths=None):
        self.masses = masses
        self.g = g
        # inertia matrices of each link referred to the COM of each link
        self.inertia = inertia
        # the relative vector of the COM with respect to the DH reference of each link
        self.r_com = r_com
        # the relative orientation of the COM with respect to the DH reference of each link
        # expressed as XYX Euler angles in mobile axes
        self.euler_com = euler_com
        # general lengths that need to be encoded
        self.lengths = lengths


class ControlParemeters():
    def __init__(self, kp, kd, ki, control_type):
        self.control_type = control_type
        self.kp = kp
        self.kd = kd
        self.ki = ki


class CoppeliaJoints():
    def __init__(self, base_name, joint_name):
        self.base_name = base_name
        self.joint_name = joint_name
        self.joint_handlers = None
