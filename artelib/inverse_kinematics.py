#!/usr/bin/env python
# encoding: utf-8
"""
inverse kinematics algotithms

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np


def delta_q(J, e, method='transpose'):
    if method == 'transpose':
        qd = delta_q_transpose(J, e)
    elif method == 'moore-penrose':
        qd = moore_penrose(J, e)
    elif method == 'moore-penrose-damped':
        qd = moore_penrose_damped(J, e)
    else:
        qd = moore_penrose_damped(J, e)
    return qd


def delta_q_transpose(J, e):
    alpha1 = np.dot(np.dot(np.dot(J, J.T), e), e)
    alpha2 = np.dot(np.dot(J, J.T), e)
    alpha2 = np.dot(alpha2, alpha2)
    alpha = alpha1/alpha2
    dq = alpha*np.dot(J.T, e)
    return dq


def moore_penrose(J, vwref):
    """
    Considers a simple joint control to behave properly in the presence of a singularity
    """
    # moore penrose pseudo inverse J^T(J*J^T)^{-1}
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
    qd = np.dot(iJ, vwref.T)
    return qd


def moore_penrose_damped(J, vwref):
    """
    Considers a simple joint control to behave properly in the presence of a singularity
    """
    manip = np.linalg.det(np.dot(J, J.T))
    # normal case --> just compute pseudo inverse we are far from a singularity
    if manip > .01**2:
        # moore penrose pseudo inverse J^T(J*J^T)^{-1}
        iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T)))
        qd = np.dot(iJ, vwref.T)
        return qd
    print('Manip is: ', manip)
    print('Close to singularity: implementing DAMPED Least squares solution')
    K = 0.01 * np.eye(np.min(J.shape))
    iJ = np.dot(J.T, np.linalg.inv(np.dot(J, J.T) + K))
    qd = np.dot(iJ, vwref.T)
    return qd





