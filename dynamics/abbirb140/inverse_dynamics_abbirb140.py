#!/usr/bin/env python
# encoding: utf-8
"""
The specific inverse dynamic for the ABBIRB140 robot. This function is called from robots/abbirb140.py

@Authors: Arturo Gil
@Time: April 2026
"""
import numpy as np
from dynamics.abbirb140.func_C import func_C
from dynamics.abbirb140.func_M import func_M
from dynamics.abbirb140.func_G import func_G

def inverse_dynamics_abbirb140(robot, q, qd, qdd):
    """
    Inverse dynamic method for the ABB IRB140 robot.
    """
    g = robot.dynamic_parameters.g
    I1x = robot.dynamic_parameters.inertia[0, 0]
    I1y = robot.dynamic_parameters.inertia[0, 1]
    I1z = robot.dynamic_parameters.inertia[0, 2]

    I2x = robot.dynamic_parameters.inertia[1, 0]
    I2y = robot.dynamic_parameters.inertia[1, 1]
    I2z = robot.dynamic_parameters.inertia[1, 2]

    I3x = robot.dynamic_parameters.inertia[2, 0]
    I3y = robot.dynamic_parameters.inertia[2, 1]
    I3z = robot.dynamic_parameters.inertia[2, 2]

    I4x = robot.dynamic_parameters.inertia[3, 0]
    I4y = robot.dynamic_parameters.inertia[3, 1]
    I4z = robot.dynamic_parameters.inertia[3, 2]

    I5x = robot.dynamic_parameters.inertia[4, 0]
    I5y = robot.dynamic_parameters.inertia[4, 1]
    I5z = robot.dynamic_parameters.inertia[4, 2]

    I6x = robot.dynamic_parameters.inertia[5, 0]
    I6y = robot.dynamic_parameters.inertia[5, 1]
    I6z = robot.dynamic_parameters.inertia[5, 2]
    # the masses
    m1 = robot.dynamic_parameters.masses[0]
    m2 = robot.dynamic_parameters.masses[1]
    m3 = robot.dynamic_parameters.masses[2]
    m4 = robot.dynamic_parameters.masses[3]
    m5 = robot.dynamic_parameters.masses[4]
    m6 = robot.dynamic_parameters.masses[5]
    # the state in shorthand
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]
    qd1 = qd[0]
    qd2 = qd[1]
    qd3 = qd[2]
    qd4 = qd[3]
    qd5 = qd[4]
    qd6 = qd[5]
    qdd1 = qdd[0]
    qdd2 = qdd[1]
    qdd3 = qdd[2]
    qdd4 = qdd[3]
    qdd5 = qdd[4]
    qdd6 = qdd[5]
    # compute M
    M = func_M(I2x,I3x,I4x,I5x,I6x,I2y,I3y,I4y,I5y,I6y,I1z,I2z,I3z,I4z,I5z,I6z,m1,m2,m3,m4,m5,m6,q1,q2,q3,q4,q5,q6)
    C = func_C(I2x,I3x,I4x,I5x,I6x,I4y,I5y,I6y,I2z,I3z,I4z,I5z,I6z,m2,m3,m4,m5,m6,q2,q3,q4,q5,q6,qd1,qd2,qd3,qd4,qd5,qd6)
    G = func_G(g,m2,m3,m4,m5,m6,q2,q3,q4,q5,q6)
    # reshape everything
    M = np.reshape(M, (6, 6))
    C = np.reshape(C, (6, 1))
    G = np.reshape(G, (6, 1))
    qdd = np.array([qdd1, qdd2, qdd3, qdd4, qdd5, qdd6])
    tau = np.matmul(M, qdd.T) + C.T + G.T
    return tau[0]