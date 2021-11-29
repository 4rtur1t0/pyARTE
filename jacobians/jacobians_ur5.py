#!/usr/bin/env python
# encoding: utf-8
"""

@Authors: Arturo Gil
@Time: April 2021

"""
import numpy as np


def eval_symbolic_jacobian_UR5(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]

    Jv = np.array([[(2183 * np.sin(q1)) / 20000 - (17 * np.cos(q1) * np.sin(q2)) / 40 + (823 * np.cos(q5) * np.sin(q1)) / 10000 - (
                1569 * np.cos(q1) * np.cos(q2) * np.sin(q3)) / 4000 - (1569 * np.cos(q1) * np.cos(q3) * np.sin(q2)) / 4000 - (
                 1893 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.sin(q4)) / 20000 - (
                 1893 * np.cos(q1) * np.cos(q2) * np.cos(q4) * np.sin(q3)) / 20000 - (
                 1893 * np.cos(q1) * np.cos(q3) * np.cos(q4) * np.sin(q2)) / 20000 + (
                 1893 * np.cos(q1) * np.sin(q2) * np.sin(q3) * np.sin(q4)) / 20000 + (
                 823 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q5)) / 10000 - (
                 823 * np.cos(q1) * np.cos(q2) * np.sin(q3) * np.sin(q4) * np.sin(q5)) / 10000 - (
                 823 * np.cos(q1) * np.cos(q3) * np.sin(q2) * np.sin(q4) * np.sin(q5)) / 10000 - (
                 823 * np.cos(q1) * np.cos(q4) * np.sin(q2) * np.sin(q3) * np.sin(q5)) / 10000, -(np.sin(q1) * (
                1893 * np.cos(q2 + q3 + q4) - 823 * np.cos(q2 + q3 + q4 + q5) + 7845 * np.cos(q2 + q3) + 8500 * np.cos(
            q2) + 823 * np.cos(q2 + q3 + q4 - q5))) / 20000, -(np.sin(q1) * (
                1893 * np.cos(q2 + q3 + q4) - 823 * np.cos(q2 + q3 + q4 + q5) + 7845 * np.cos(q2 + q3) + 823 * np.cos(
            q2 + q3 + q4 - q5))) / 20000, -np.sin(q1) * (
                 (1893 * np.cos(q2 + q3 + q4)) / 20000 - (823 * np.cos(q2 + q3 + q4 + q5)) / 20000 + (
                     823 * np.cos(q2 + q3 + q4 - q5)) / 20000),
            (823 * np.cos(q1) * np.sin(q5)) / 10000 + (823 * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q1)) / 10000 - (
                 823 * np.cos(q2) * np.cos(q5) * np.sin(q1) * np.sin(q3) * np.sin(q4)) / 10000 - (
                 823 * np.cos(q3) * np.cos(q5) * np.sin(q1) * np.sin(q2) * np.sin(q4)) / 10000 - (
                 823 * np.cos(q4) * np.cos(q5) * np.sin(q1) * np.sin(q2) * np.sin(q3)) / 10000, 0],
            [(1893 * np.sin(q1) * np.sin(q2) * np.sin(q3) * np.sin(q4)) / 20000 - (823 * np.cos(q1) * np.cos(q5)) / 10000 - (
                17 * np.sin(q1) * np.sin(q2)) / 40 - (1569 * np.cos(q2) * np.sin(q1) * np.sin(q3)) / 4000 - (
                 1569 * np.cos(q3) * np.sin(q1) * np.sin(q2)) / 4000 - (1893 * np.cos(q2) * np.cos(q3) * np.sin(q1) * np.sin(q4)) / 20000 - (
                 1893 * np.cos(q2) * np.cos(q4) * np.sin(q1) * np.sin(q3)) / 20000 - (
                 1893 * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 20000 - (2183 * np.cos(q1)) / 20000 + (
                 823 * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q5)) / 10000 - (
                 823 * np.cos(q2) * np.sin(q1) * np.sin(q3) * np.sin(q4) * np.sin(q5)) / 10000 - (
                 823 * np.cos(q3) * np.sin(q1) * np.sin(q2) * np.sin(q4) * np.sin(q5)) / 10000 - (
                 823 * np.cos(q4) * np.sin(q1) * np.sin(q2) * np.sin(q3) * np.sin(q5)) / 10000, (np.cos(q1) * (
                1893 * np.cos(q2 + q3 + q4) - 823 * np.cos(q2 + q3 + q4 + q5) + 7845 * np.cos(q2 + q3) + 8500 * np.cos(
            q2) + 823 * np.cos(q2 + q3 + q4 - q5))) / 20000, (np.cos(q1) * (
                1893 * np.cos(q2 + q3 + q4) - 823 * np.cos(q2 + q3 + q4 + q5) + 7845 * np.cos(q2 + q3) + 823 * np.cos(
            q2 + q3 + q4 - q5))) / 20000, np.cos(q1) * (
                 (1893 * np.cos(q2 + q3 + q4)) / 20000 - (823 * np.cos(q2 + q3 + q4 + q5)) / 20000 + (
                     823 * np.cos(q2 + q3 + q4 - q5)) / 20000),
     (823 * np.sin(q1) * np.sin(q5)) / 10000 - (823 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5)) / 10000 + (
                 823 * np.cos(q1) * np.cos(q2) * np.cos(q5) * np.sin(q3) * np.sin(q4)) / 10000 + (
                 823 * np.cos(q1) * np.cos(q3) * np.cos(q5) * np.sin(q2) * np.sin(q4)) / 10000 + (
                 823 * np.cos(q1) * np.cos(q4) * np.cos(q5) * np.sin(q2) * np.sin(q3)) / 10000, 0],
    [0, (823 * np.sin(q2 + q3 + q4 + q5)) / 20000 - (1893 * np.sin(q2 + q3 + q4)) / 20000 - (1569 * np.sin(q2 + q3)) / 4000 - (
                17 * np.sin(q2)) / 40 - (823 * np.sin(q2 + q3 + q4 - q5)) / 20000,
     (823 * np.sin(q2 + q3 + q4 + q5)) / 20000 - (1893 * np.sin(q2 + q3 + q4)) / 20000 - (1569 * np.sin(q2 + q3)) / 4000 - (
                 823 * np.sin(q2 + q3 + q4 - q5)) / 20000,
     (823 * np.sin(q2 + q3 + q4 + q5)) / 20000 - (1893 * np.sin(q2 + q3 + q4)) / 20000 - (
                 823 * np.sin(q2 + q3 + q4 - q5)) / 20000,
     (823 * np.sin(q2 + q3 + q4 + q5)) / 20000 + (823 * np.sin(q2 + q3 + q4 - q5)) / 20000, 0]])

    Jw = np.array([[0, -np.cos(q1), -np.cos(q1), -np.cos(q1), np.cos(q1 + q2 + q3 + q4) / 2 - np.cos(q2 - q1 + q3 + q4) / 2,
     np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q5) - np.cos(q1) * np.cos(q5) - np.cos(q2) * np.sin(q1) * np.sin(q3) * np.sin(q4) * np.sin(
         q5) - np.cos(q3) * np.sin(q1) * np.sin(q2) * np.sin(q4) * np.sin(q5) - np.cos(q4) * np.sin(q1) * np.sin(q2) * np.sin(q3) * np.sin(q5)],
    [0, -np.sin(q1), -np.sin(q1), -np.sin(q1), np.sin(q1 + q2 + q3 + q4) / 2 + np.sin(q2 - q1 + q3 + q4) / 2,
     np.cos(q1) * np.cos(q2) * np.sin(q3) * np.sin(q4) * np.sin(q5) - np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q5) - np.cos(q5) * np.sin(
         q1) + np.cos(q1) * np.cos(q3) * np.sin(q2) * np.sin(q4) * np.sin(q5) + np.cos(q1) * np.cos(q4) * np.sin(q2) * np.sin(q3) * np.sin(q5)],
    [1, 0, 0, 0, np.cos(q2 + q3 + q4), np.cos(q2 + q3 + q4 - q5) / 2 - np.cos(q2 + q3 + q4 + q5) / 2]])

    J = np.vstack((Jv, Jw))
    return J, Jv, Jw




