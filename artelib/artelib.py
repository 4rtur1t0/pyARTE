#!/usr/bin/env python
# encoding: utf-8
"""
A number of useful functions
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np

def compute_w_between_orientations(orientation, targetorientation):
    R1 = euler2Rot(orientation)
    R2 = euler2Rot(targetorientation)
    Q1 = R2quaternion(R1)
    Q2 = R2quaternion(R2)
    # compute the angular speed w that rotates from Q1 to Q2
    w = angular_w_between_quaternions(Q1, Q2, 1)
    return w



def R2quaternion(R):
    """
    Transforms a rotation matrix R into a quaternion
    """
    Q = np.zeros(4)
    Q[0] = np.sqrt(np.trace(R))/2

    Lx = R[2, 1] - R[1, 2]
    Ly = R[0, 2] - R[2, 0]
    Lz = R[1, 0] - R[0, 1]

    if (R[0, 0] >= R[1, 1]) and (R[0, 0] >= R[2, 2]):
        Lx1 = R[0, 0] - R[1, 1] - R[2, 2] + 1
        Ly1 = R[1, 0] + R[0, 1]
        Lz1 = R[2, 0] + R[0, 2]
    elif R[1,1] >= R[2,2]:
        Lx1 = R[1, 0] + R[0, 1]
        Ly1 = R[1, 1] - R[0, 0] - R[2, 2] + 1
        Lz1 = R[2, 1] + R[1, 2]
    else:
        Lx1 = R[2, 0] + R[0, 2]
        Ly1 = R[2, 1] + R[1, 2]
        Lz1 = R[2, 2] - R[0, 0] - R[1, 1] + 1

    if (Lx >= 0) or (Ly >= 0) or (Lz >= 0):
        Lx = Lx + Lx1
        Ly = Ly + Ly1
        Lz = Lz + Lz1
    else:
        Lx = Lx - Lx1
        Ly = Ly - Ly1
        Lz = Lz - Lz1

    if np.linalg.norm([Lx, Ly, Lz]) == 0:
        Q = np.array([1, 0, 0, 0])
    else:
        s = np.sqrt(1-Q[0]**2)/np.linalg.norm(np.array([Lx, Ly, Lz]))
        Q2 = s*np.array([Lx, Ly, Lz])
        Q[1] = Q2[0]
        Q[2] = Q2[1]
        Q[3] = Q2[2]
    return Q

def angular_w_between_quaternions(Q0, Q1, total_time):
    epsilon_len = 0.000001
    # Let's first find quaternion q so q*q0=q1 it is q=q1/q0
    # For unit length quaternions, you can use q=q1*Conj(q0)
    Q = qprod(Q1, qconj(Q0))
    # To find rotation velocity that turns by q during time Dt you need to
    # convert quaternion to axis angle using something like this:
    length = np.sqrt(Q[1]**2 + Q[2]**2 + Q[3]**2)
    if length > epsilon_len:
        angle = 2*np.arctan2(length, Q[0])
        axis = np.array([Q[1], Q[2], Q[3]])
        axis = np.dot(1/length, axis)
    else:
        angle = 0
        axis = np.array([1, 0, 0])
    w = np.dot(angle/total_time, axis)
    return w

def qprod(q1, q2):
    """
    quaternion product
    """
    a = q1[0]
    b = q2[0]
    v1 = q1[1:4]
    v2 = q2[1:4]
    s = a*b - np.dot(v1, v2.T)
    v = np.dot(a, v2) + np.dot(b, v1) + np.cross(v1, v2)
    Q = np.hstack((s, v))
    return Q

def qconj(q):
    s = q[0]
    v = q[1:4]
    Q = np.hstack((s, -v))
    return Q

def euler2Rot(abg):
    calpha = np.cos(abg[0])
    salpha = np.sin(abg[0])
    cbeta = np.cos(abg[1])
    sbeta = np.sin(abg[1])
    cgamma = np.cos(abg[2])
    sgamma = np.sin(abg[2])
    Rx = np.array([[1, 0, 0], [0, calpha, -salpha], [0, salpha, calpha]])
    Ry = np.array([[cbeta, 0, sbeta], [0, 1, 0], [-sbeta, 0, cbeta]])
    Rz = np.array([[cgamma, -sgamma, 0], [sgamma, cgamma, 0], [0, 0, 1]])

    R = np.matmul(Rx, Ry)
    R = np.matmul(R, Rz)
    return R