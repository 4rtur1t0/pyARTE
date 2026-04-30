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
    R = orientation.R()
    R = R.toarray()
    T[0:3, 0:3] = R
    T[3, 3] = 1
    T[0:3, 3] = np.array(position).T
    return T


def normalize(q):
    norma = np.linalg.norm(q)
    if norma > 0:
        return q/norma
    else:
        return q


def normalize_angle(eul):
    """
    Normalize angles in array to [-pi, pi]
    """
    e = []
    for i in range(len(eul)):
        e.append(np.arctan2(np.sin(eul[i]), np.cos(eul[i])))
    return e


#
# def mod_sign(x):
#     """
#        modified  version of sign() function as per   the    paper
#         sign(x) = 1 if x >= 0
#     """
#     if x >= 0:
#         return 1
#     else:
#         return -1


def angular_w_between_quaternions(Q0, Q1, total_time):
    """
    Compute the angular velocity between two quaternions
    """
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
    try:
        Jp = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        P = I - np.dot(Jp, J)
    except np.linalg.LinAlgError:
        P = I
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


def saturate_qi(q, qmin, qmax):
    deltaq = np.pi/20
    if q > (qmax - deltaq):
        q = qmax - deltaq
    elif q < (qmin + deltaq):
        q = qmin + deltaq
    return q


def w_lateral(q, qmin, qmax):
    """
    The evaluation of a lateral function that avoids going into the range limits of the joints.
    A delta_q is considered in order to avoid the singularity if q[i]==qmax[i] of q[i]==qmax[i]
    """
    n = len(q)
    w = 0
    for i in range(n):
        qi = saturate_qi(q[i], qmin[i], qmax[i])
        num = (qmax[i]-qmin[i])
        den = (qmax[i]-qi)*(qi-qmin[i])
        if den > 0.0:
            w += num/den
        else:
            w += 0.0
    w = w/2/n
    return w


def diff_w_lateral(q, qmin, qmax):
    """
    The function computes the differential of the secondary function w_lateral
    """
    n = len(q)
    dw = []
    for i in range(n):
        qi = saturate_qi(q[i], qmin[i], qmax[i])
        dwi_num = (qmin[i] - qmax[i])*(-2*qi + qmax[i] + qmin[i])
        dwi_den = ((qi - qmax[i]) * (qi - qmin[i]))**2
        dwi = dwi_num/dwi_den
        if np.isinf(dwi) or np.isnan(dwi):
            dwi = 0.0
        dw.append(dwi)
    return np.array(dw)


def minimize_w_lateral(J, q, qmax, qmin):
    qd0 = diff_w_lateral(q, qmax, qmin)
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

