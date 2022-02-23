#!/usr/bin/env python
# encoding: utf-8
"""

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np


def eval_symbolic_jacobian_planar_4dof(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]

    Jv = np.array([[- np.sin(q1 + q2 + q3 + q4)/10 -np.sin(q1 + q2 + q3)/5 - (3*np.sin(q1 + q2))/10 - (2*np.sin(q1))/5, -np.sin(q1 + q2 + q3)/5 - np.sin(q1 + q2 + q3 + q4)/10 - (3*np.sin(q1 + q2))/10, - np.sin(q1 + q2 + q3)/5 - np.sin(q1 + q2 + q3 + q4)/10, -np.sin(q1 + q2 + q3 + q4)/10],
                  [np.cos(q1 + q2 + q3)/5 + np.cos(q1 + q2 + q3 + q4)/10 + (3*np.cos(q1 + q2))/10 + (2*np.cos(q1))/5,   np.cos(q1 + q2 + q3)/5 + np.cos(q1 + q2 + q3 + q4)/10 + (3*np.cos(q1 + q2))/10,   np.cos(q1 + q2 + q3)/5 + np.cos(q1 + q2 + q3 + q4)/10,  np.cos(q1 + q2 + q3 + q4)/10],
                  [0,  0,  0,  0]])
    Jw = np.array([[0, 0, 0, 0],
                   [0, 0, 0, 0],
                   [1, 1, 1, 1]])
    J = np.vstack((Jv, Jw))
    return J, Jv, Jw


def eval_symbolic_T_planar4dof(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    T = np.array([[np.cos(q1 + q2 + q3 + q4), -np.sin(q1 + q2 + q3 + q4),  0, np.cos(q1 + q2 + q3)/5 + np.cos(q1 + q2 + q3 + q4)/10 + (3*np.cos(q1 + q2))/10 + (2*np.cos(q1))/5],
                  [np.sin(q1 + q2 + q3 + q4),  np.cos(q1 + q2 + q3 + q4), 0, np.sin(q1 + q2 + q3)/5 + np.sin(q1 + q2 + q3 + q4)/10 + (3*np.sin(q1 + q2))/10 + (2*np.sin(q1))/5],
                  [0,                       0,               1, 0],
                  [0,                       0,               0, 1]])
    return T



