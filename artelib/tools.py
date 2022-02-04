#!/usr/bin/env python
# encoding: utf-8
"""
A number of useful functions
@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np


def buildT(position, orientation):
    T = np.zeros((4, 4))
    # R = euler2rot(orientation)
    R = orientation.R()
    T[0:3, 0:3] = R
    T[3, 3] = 1
    T[0:3, 3] = np.array(position).T
    return T


# def buildTQ(position, orientationQ):
#     T = np.zeros((4, 4))
#     # R = quaternion2rot(orientationQ)
#     T[0:3, 0:3] = R
#     T[3, 3] = 1
#     T[0:3, 3] = np.array(position).T
#     return T

def compute_w_between_orientations(orientation, targetorientation):
    # R1 = euler2rot(orientation.R
    # R2 = euler2rot(targetorientation)
    R1 = orientation.R()
    R2 = targetorientation.R()
    Q1 = rot2quaternion(R1)
    Q2 = rot2quaternion(R2)
    # compute the angular speed w that rotates from Q1 to Q2
    w = angular_w_between_quaternions(Q1, Q2, 1)
    return w


def compute_w_between_R(Rcurrent, Rtarget, total_time=1):
    R1 = Rcurrent[0:3, 0:3]
    R2 = Rtarget[0:3, 0:3]
    Q1 = rot2quaternion(R1)
    Q2 = rot2quaternion(R2)
    # compute the angular speed w that rotates from Q1 to Q2
    w = angular_w_between_quaternions(Q1, Q2, total_time=total_time)
    return w


def compute_e_between_R(Rcurrent, Rtarget):
    R1 = Rcurrent[0:3, 0:3]
    R2 = Rtarget[0:3, 0:3]

    ne = R1[:, 0]
    se = R1[:, 1]
    ae = R1[:, 2]

    nd = R2[:, 0]
    sd = R2[:, 1]
    ad = R2[:, 2]
    e = np.cross(ne, nd) + np.cross(se, sd) + np.cross(ae, ad)
    e = 0.5*e
    return e


def compute_kinematic_errors(Tcurrent, Ttarget):
    """
    Compute the error
    """
    # current position of the end effector and target position
    p_current = Tcurrent[0:3, 3]
    p_target = Ttarget[0:3, 3]
    e1 = np.array(p_target - p_current)
    error_dist = np.linalg.norm(e1)
    e2 = compute_e_between_R(Tcurrent, Ttarget)
    error_orient = np.linalg.norm(e2)
    e = np.hstack((e1, e2))
    return e, error_dist, error_orient


def quaternion2rot(Q):
    qw = Q[0]
    qx = Q[1]
    qy = Q[2]
    qz = Q[3]
    R = np.eye(3)
    R[0, 0] = 1 - 2 * qy**2 - 2 * qz**2
    R[0, 1] = 2 * qx * qy - 2 * qz * qw
    R[0, 2] = 2 * qx * qz + 2 * qy * qw
    R[1, 0] = 2 * qx * qy + 2 * qz * qw
    R[1, 1] = 1 - 2*qx**2 - 2*qz**2
    R[1, 2] = 2 * qy * qz - 2 * qx * qw
    R[2, 0] = 2 * qx * qz - 2 * qy * qw
    R[2, 1] = 2 * qy * qz + 2 * qx * qw
    R[2, 2] = 1 - 2 * qx**2 - 2 * qy**2
    return R


def rot2quaternion(R):
    """
    rot2quaternion(R)
    Computes a quaternion Q from a rotation matrix R.

    This implementation has been translated from The Robotics Toolbox for Matlab (Peter  Corke),
    https://github.com/petercorke/spatial-math
    """
    R = R[0:3, 0:3]
    s = np.sqrt(np.trace(R) + 1) / 2.0
    kx = R[2, 1] - R[1, 2] # Oz - Ay
    ky = R[0, 2] - R[2, 0] # Ax - Nz
    kz = R[1, 0] - R[0, 1] # Ny - Ox

    # equation(7)
    k = np.argmax(np.diag(R))
    if k == 0: # Nx dominates
        kx1 = R[0, 0] - R[1, 1] - R[2, 2] + 1 # Nx - Oy - Az + 1
        ky1 = R[1, 0] + R[0, 1] # Ny + Ox
        kz1 = R[2, 0] + R[0, 2]  # Nz + Ax
        sgn = mod_sign(kx)
    elif k == 1: # Oy dominates
        kx1 = R[1, 0] + R[0, 1] # % Ny + Ox
        ky1 = R[1, 1] - R[0, 0] - R[2, 2] + 1  # Oy - Nx - Az + 1
        kz1 = R[2, 1] + R[1, 2] # % Oz + Ay
        sgn = mod_sign(ky)
    elif k == 2: # Az dominates
        kx1 = R[2, 0] + R[0, 2] # Nz + Ax
        ky1 = R[2, 1] + R[1, 2] # Oz + Ay
        kz1 = R[2, 2] - R[0, 0] - R[1, 1] + 1 # Az - Nx - Oy + 1
        # add = (kz >= 0)
        sgn = mod_sign(kz)
    # equation(8)
    kx = kx + sgn * kx1
    ky = ky + sgn * ky1
    kz = kz + sgn * kz1

    nm = np.linalg.norm([kx, ky, kz])
    if nm == 0:
        # handle special case of null quaternion
        Q = np.array([1, 0, 0, 0])
    else:
        v = np.dot(np.sqrt(1 - s**2)/nm, np.array([kx, ky, kz])) # equation(10)
        Q = np.hstack((s, v))
    return Q


def mod_sign(x):
    """
       modified  version of sign() function as per   the    paper
        sign(x) = 1 if x >= 0
    """
    if x >= 0:
        return 1
    else:
        return -1


def angular_w_between_quaternions(Q0, Q1, total_time):
    epsilon_len = 0.01000
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


def euler2rot(abg):
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


def rot2euler(R):
    """
    Computes Euler angles for the expression Rx(alpha)Ry(beta)Rz(gamma)
    Caution: convention is XYZ
    :param R:
    :return:
    """
    R = R[0:3, 0:3]
    # caution, c-like indexes in python!
    sbeta = R[0, 2]
    if abs(sbeta) == 1.0:
        # degenerate case in which sin(beta)=+-1 and cos(beta)=0
        # arbitrarily set alpha to zero
        alpha = 0.0
        beta = np.arcsin(sbeta)
        gamma = np.arctan2(R[1, 1], R[1, 0])
    else:
        # standard way to compute alpha beta and gamma
        alpha = -np.arctan2(R[1, 2], R[2, 2])
        beta = np.arctan2(np.cos(alpha) * R[0, 2], R[2, 2])
        gamma = -np.arctan2(R[0, 1], R[0, 0])
    return [alpha, beta, gamma]


def euler2q(abg):
    R = euler2rot(abg=abg)
    Q = rot2quaternion(R)
    return Q

def q2euler(Q):
    R = quaternion2rot(Q)
    abg = rot2euler(R)
    return abg


def slerp(Q1, Q2, t):
    """
    Interpolates between quaternions Q1 and Q2, given a fraction 1
    """
    cth = np.dot(Q1, Q2)
    th = np.arccos(cth)
    if np.abs(th) > 0:
        Q = Q1*np.sin((1-t)*th)/np.sin(th) + Q2*np.sin(t*th)/np.sin(th)
        return Q
    # if th == 0, dividing by zero, just return Q1
    else:
        return Q1


def null_space(J, n):
    """
    Obtain a unit vector in the direction of the null space using the SVD method.
    Consider m degrees of liberty in the task.
    Return the column n in the null space
    """
    u, s, vh = np.linalg.svd(J, full_matrices=True)
    qd = vh.T[:, n]
    return qd


def null_space_projector(J):
    n = J.shape[1]
    I = np.eye(n)
    # pseudo inverse moore-penrose
    # here, do not use pinv
    Jp = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
    P = I - np.dot(Jp, J)
    return P


def w_central(q, qcentral, K):
    w = 0
    for i in range(0, len(qcentral)):
        w += K[i]*(q[i]-qcentral[i])**2
    return w


def diff_w_central(q, qcentral, K):
    dw = []
    for i in range(0, len(qcentral)):
        dwi = K[i]*(q[i]-qcentral[i])
        dw.append(dwi)
    return np.array(dw)


def minimize_w_central(J, q, qc, K):
    qd0 = diff_w_central(q, qc, K)
    qd0 = np.dot(-1.0, qd0)
    P = null_space_projector(J)
    qdb = np.dot(P, qd0)
    norma = np.linalg.norm(qdb)
    if np.isnan(np.sum(qdb)):
        return np.zeros(len(q))
    if norma > 0.0001:
        return qdb / norma
    else:
        return qdb


def w_lateral(q, qmax, qmin):
    n = len(q)
    w = 0
    for i in range(n):
        num = qmax[i]-qmin[i]
        den = (qmax[i]-q[i])*(q[i]-qmin[i])
        if den > 0:
            w += num/den
    w = w/2/n
    return w


def diff_w_lateral(q, qmax, qmin):
    n = len(q)
    dw = []
    for i in range(n):
        dwi = -(qmin[i]-qmax[i])/((q[i]-qmin[i])*(q[i]-qmax[i])**2)-(qmin[i] - qmax[i]) / ((q[i] - qmax[i])*(q[i] - qmin[i])**2)
        if np.isinf(dwi):
            dwi = 0
        dw.append(dwi)
    return np.array(dw)


def minimize_w_lateral(J, q, qmax, qmin):
    qd0 = diff_w_lateral(q, qmax, qmin)
    qd0 = np.dot(-1.0, qd0)
    P = null_space_projector(J)
    qdb = np.dot(P, qd0)
    norma = np.linalg.norm(qdb)
    if norma > 0.0001:
        return qdb / norma
    else:
        return qdb

