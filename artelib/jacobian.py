#!/usr/bin/env python
# encoding: utf-8
"""
Jacobian manipulator related functions.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np


def compute_jacobian_manipulator_function(robot, q):
    # T = self.serialrobot.directkinematics(q)
    # number  of  DOF
    n = len(q)
    # base rotation matrix
    # R0 = np.eye(3)
    R0 = robot.serial_parameters.T0[0:3, 0:3]
    # Z vector on   each     reference     system
    z0 = np.array([0, 0, 1])

    # compute zi vectors. We first  start   by computing the   zi vectors   of    each    % reference    system
    # from z0, z1, z2..., z_{n - 1}. Note that the last  rotational (translational)  joint  acts   on z_{n - 1}
    # Array  to   store    every     z     vector
    z = []

    # this loop  computes vectors from z0 to z_{n - 1}
    for i in range(n):
        zi = np.dot(R0, z0)
        # store the  vector in a list
        z.append(zi)
        # compute  the  DH    transformation   matrix   from system  i - 1     to    system    i
        A = robot.serial_parameters.dh(q, i)
        # obtain  now    the    global transformation    by     postmultiplying     the rotational
        #     part. In     the     following    iteration   we     include    the    last    DH transformation
        R0 = np.dot(R0, A[0:3, 0:3])
    z = np.array(z)
    z = z.T

    # compute  p{i - 1}n *  vectors   that  represent  the   position   of   the    end effector in the  {i - 1}
    #  reference    system. Please  note  that  the  following code is not optimal (at all), the  total
    # transformation  matrix    should  be  computed in the    loop   above. However, we  compute  it   now   using
    # the directkinematic function for this particular  robot. Again, the    DH  matrices   are   computed    again.
    T = robot.serial_parameters.directkinematics(q)
    pn = np.zeros((3, n))
    Ti = robot.serial_parameters.T0
    v = Ti[0:3, 3]
    for i in range(n):
        pn[:, i] = T[0: 3, 3] - v
        A = robot.serial_parameters.dh(q, i)
        Ti = np.dot(Ti, A)
        # v is the origin  of  the   ith  reference  system in base   coordinates
        v = Ti[0:3, 3]
    # nowcompute conventional Jacobian
    J = np.zeros((6, n))
    for i in range(n):
        # rotational
        if robot.serial_parameters.get_link_type(i) == 'R':
            J[:, i] = np.concatenate((np.cross(z[:, i], pn[:, i]), z[:, i]))
        else: # translational   joint
            J[:, i] = np.concatenate((z[:, i], np.zeros((3, 1))))
    return J, J[0:3, :], J[3:6, :]

