#!/usr/bin/env python
# encoding: utf-8
"""
Simple path planning functions.

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np
from artelib.orientation import Quaternion
from artelib.tools import euler2rot, rot2quaternion, slerp, rot2euler, quaternion2rot, buildT


class Node():
    def __init__(self, p0, p1, p2, ps):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        self.ps = ps
        self.young = 0.1
        self.mass = 100.0
        self.db = np.linalg.norm(p0-p1)

    def iteration_step(self):
        fa = self.compute_attraction(self.p1)
        fb = self.compute_attraction(self.p2)
        fs = self.compute_repulsion()
        dp = (1/self.mass)*(fa+fb+fs)
        self.p0 = self.p0 + dp

    def compute_attraction(self, p):
        """
        attraction p0 p1
        """
        d = np.linalg.norm(self.p0-p)
        f = self.young*(p-self.p0)*(self.db - d)
        return f

    def potential(self, r):
        K = 0.4
        rs = 0.1  # radius of the sphere
        rmax = 0.3
        if r < rs:
            r = rs
        p = K * (1 / r - 1 / rmax)
        if p < 0:
            p = 0
        return p

    def compute_repulsion(self):
        u = self.p0 - self.ps
        r = np.linalg.norm(u)
        if r > 0.0:
            u = u / r
        p = self.potential(r)
        frep = np.dot(p, u)
        return frep



def generate_target_positions_on_line(p_current, p_target, vmax, delta_time=0.05):
    u = p_target - p_current
    d = np.linalg.norm(p_target - p_current)
    if d > 0:
        u = u / d
    total_time = d / vmax
    n = total_time / delta_time
    n = np.ceil(n)
    delta = d / n

    target_positions = [p_current]
    for i in range(1, int(n) + 1):
        pi = p_current + i * delta * u
        target_positions.append(pi)
    return target_positions


def generate_target_orientations_on_line(abc_current, abc_target, n):
    abc_current = np.array(abc_current)
    abc_target = np.array(abc_target)
    R1 = euler2rot(abc_current)
    R2 = euler2rot(abc_target)
    Q1 = rot2quaternion(R1)
    Q2 = rot2quaternion(R2)

    tt = np.linspace(0, 1, n)
    target_orientations = []
    for t in tt:
        Q = slerp(Q1, Q2, t)
        R = quaternion2rot(Q)
        abc = rot2euler(R)
        target_orientations.append(abc)
    return target_orientations


def generate_target_orientations_on_line_Q(Q1, Q2, n):
    tt = np.linspace(0, 1, n)
    target_orientations = []
    for t in tt:
        Q = slerp(Q1, Q2, t)
        Q = Quaternion(Q)
        target_orientations.append(Q)
    return target_orientations


def generate_on_line_obstacles(robot, target_position, target_orientation, sphere_position, q0, vmax=1.0):
        """
        Generate a series of points on the line.

        """
        Ttarget = buildT(target_position, target_orientation)
        Ti = robot.direct_kinematics(q0)
        Qcurrent = rot2quaternion(Ti)
        Qtarget = target_orientation.Q()

        p_current = Ti[0:3, 3]
        p_target = Ttarget[0:3, 3]
        target_positions = generate_target_positions_on_line(p_current, p_target, vmax=vmax, delta_time=0.05)
        # generating quaternions on the line. Use SLERP to interpolate between quaternions
        target_orientations = generate_target_orientations_on_line_Q(Qcurrent, Qtarget,
                                                                   len(target_positions))

