#!/usr/bin/env python
# encoding: utf-8
"""
Simbolic functions to represent the Jacobian and direct kinematics of the KUKA LBR robot.

@Authors: Arturo Gil
@Time: April 2021
"""
import numpy as np

def eval_symbolic_jacobian_KUKALBR(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]

    Jv = np.array([[(111 * np.cos(q6) * (np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 - (
        21 * np.sin(q1) * np.sin(q2)) / 50 - (111 * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (
    np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)))) / 1000 + (
                 2 * np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1))) / 5 - (
                 2 * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 5, np.cos(q1) * ((21 * np.cos(q2)) / 50 + (2 * np.cos(q2) * np.cos(q4)) / 5 + (
                111 * np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(
            q5))) / 1000 + (111 * np.cos(q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) / 1000 + (
                                                                              2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5),
     (2 * np.cos(q3) * np.sin(q1) * np.sin(q4)) / 5 + (2 * np.cos(q1) * np.cos(q2) * np.sin(q3) * np.sin(q4)) / 5 + (
                 111 * np.cos(q3) * np.cos(q6) * np.sin(q1) * np.sin(q4)) / 1000 + (
                 111 * np.sin(q1) * np.sin(q3) * np.sin(q5) * np.sin(q6)) / 1000 + (
                 111 * np.cos(q1) * np.cos(q2) * np.cos(q6) * np.sin(q3) * np.sin(q4)) / 1000 - (
                 111 * np.cos(q1) * np.cos(q2) * np.cos(q3) * np.sin(q5) * np.sin(q6)) / 1000 - (
                 111 * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q1) * np.sin(q6)) / 1000 - (
                 111 * np.cos(q1) * np.cos(q2) * np.cos(q4) * np.cos(q5) * np.sin(q3) * np.sin(q6)) / 1000,
     - (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)) * ((2 * np.cos(q2) * np.cos(q4)) / 5 + (111 * np.sin(q6) * (
                 np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                        111 * np.cos(q6) * (
                                                                            np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(
                                                                        q4))) / 1000 + (
                                                                        2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5) - np.sin(
         q2) * np.sin(q3) * ((111 * np.cos(q6) * (
                 np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 - (
                                      111 * np.sin(q6) * (np.cos(q5) * (
                                          np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(
                                      q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(
                                  q3)))) / 1000 + (
                                      2 * np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1))) / 5 - (
                                      2 * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 5), -(111 * np.sin(q6) * (
                np.cos(q3) * np.cos(q5) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.cos(q5) * np.sin(q3) + np.cos(q1) * np.sin(q2) * np.sin(q4) * np.sin(
            q5) - np.cos(q4) * np.sin(q1) * np.sin(q3) * np.sin(q5) + np.cos(q1) * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q5))) / 1000,
     - (np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) - np.cos(q5) * np.sin(q2) * np.sin(q3)) * ((111 * np.cos(q6) * (
                 np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 - (
                                                                                                                  111 * np.sin(
                                                                                                              q6) * (
                                                                                                                              np.cos(q5) * (
                                                                                                                                  np.cos(q4) * (
                                                                                                                                      np.cos(q1) * np.sin(
                                                                                                                                  q3) + np.cos(
                                                                                                                                  q2) * np.cos(
                                                                                                                                  q3) * np.sin(
                                                                                                                                  q1)) + np.sin(
                                                                                                                              q1) * np.sin(
                                                                                                                              q2) * np.sin(
                                                                                                                              q4)) + np.sin(
                                                                                                                          q5) * (
                                                                                                                                          np.cos(q1) * np.cos(
                                                                                                                                      q3) - np.cos(
                                                                                                                                      q2) * np.sin(
                                                                                                                                      q1) * np.sin(
                                                                                                                                      q3)))) / 1000) - (
                 np.sin(q5) * (
                     np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) - np.cos(
             q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3))) * ((111 * np.sin(q6) * (
                 np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                                     111 * np.cos(q6) * (
                                                                                         np.cos(q2) * np.cos(q4) + np.cos(
                                                                                     q3) * np.sin(q2) * np.sin(q4))) / 1000),
     0],
    [(21 * np.cos(q1) * np.sin(q2)) / 50 + (111 * np.cos(q6) * (np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2))) / 1000 - (
    111 * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
             q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))) / 1000 + (
                 2 * np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3))) / 5 + (
                 2 * np.cos(q1) * np.cos(q4) * np.sin(q2)) / 5, np.sin(q1) * ((21 * np.cos(q2)) / 50 + (2 * np.cos(q2) * np.cos(q4)) / 5 + (
                111 * np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(
            q5))) / 1000 + (111 * np.cos(q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) / 1000 + (2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5),
     (2 * np.cos(q2) * np.sin(q1) * np.sin(q3) * np.sin(q4)) / 5 - (111 * np.cos(q1) * np.cos(q3) * np.cos(q6) * np.sin(q4)) / 1000 - (
                 2 * np.cos(q1) * np.cos(q3) * np.sin(q4)) / 5 - (111 * np.cos(q1) * np.sin(q3) * np.sin(q5) * np.sin(q6)) / 1000 + (
                 111 * np.cos(q1) * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q6)) / 1000 + (
                 111 * np.cos(q2) * np.cos(q6) * np.sin(q1) * np.sin(q3) * np.sin(q4)) / 1000 - (
                 111 * np.cos(q2) * np.cos(q3) * np.sin(q1) * np.sin(q5) * np.sin(q6)) / 1000 - (
                 111 * np.cos(q2) * np.cos(q4) * np.cos(q5) * np.sin(q1) * np.sin(q3) * np.sin(q6)) / 1000,
     - (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)) * ((2 * np.cos(q2) * np.cos(q4)) / 5 + (111 * np.sin(q6) * (
                 np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
                                                                        111 * np.cos(q6) * (
                                                                            np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(
                                                                        q4))) / 1000 + (
                                                                        2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5) - np.sin(
         q2) * np.sin(q3) * ((111 * np.cos(q6) * (
                 np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2))) / 1000 - (
                                      111 * np.sin(q6) * (np.cos(q5) * (
                                          np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(
                                      q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(
                                  q3)))) / 1000 + (
                                      2 * np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3))) / 5 + (
                                      2 * np.cos(q1) * np.cos(q4) * np.sin(q2)) / 5), -(111 * np.sin(q6) * (
                np.cos(q2) * np.cos(q5) * np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q3) * np.cos(q5) + np.cos(q1) * np.cos(q4) * np.sin(q3) * np.sin(
            q5) + np.sin(q1) * np.sin(q2) * np.sin(q4) * np.sin(q5) + np.cos(q2) * np.cos(q3) * np.cos(q4) * np.sin(q1) * np.sin(q5))) / 1000,
     - (np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) - np.cos(q5) * np.sin(q2) * np.sin(q3)) * ((111 * np.cos(q6) * (
                 np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2))) / 1000 - (
      111 * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3))
                                        - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))) / 1000) -
     (np.sin(q5) * (np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) - np.cos(
             q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3))) * ((111 * np.sin(q6) * (
                 np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5))) / 1000 + (
        111 * np.cos(q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) / 1000),  0],
    [0, (2 * np.cos(q2) * np.cos(q3) * np.sin(q4)) / 5 - (2 * np.cos(q4) * np.sin(q2)) / 5 - (21 * np.sin(q2)) / 50 - (
                111 * np.cos(q4) * np.cos(q6) * np.sin(q2)) / 1000 + (111 * np.cos(q2) * np.cos(q3) * np.cos(q6) * np.sin(q4)) / 1000 + (
                 111 * np.cos(q2) * np.sin(q3) * np.sin(q5) * np.sin(q6)) / 1000 - (
                 111 * np.cos(q5) * np.sin(q2) * np.sin(q4) * np.sin(q6)) / 1000 - (
                 111 * np.cos(q2) * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.sin(q6)) / 1000, -(np.sin(q2) * (
                400 * np.sin(q3) * np.sin(q4) + 111 * np.cos(q6) * np.sin(q3) * np.sin(q4) - 111 * np.cos(q3) * np.sin(q5) * np.sin(
            q6) - 111 * np.cos(q4) * np.cos(q5) * np.sin(q3) * np.sin(q6))) / 1000,
     (2 * np.cos(q3) * np.cos(q4) * np.sin(q2)) / 5 - (2 * np.cos(q2) * np.sin(q4)) / 5 - (
                 111 * np.cos(q2) * np.cos(q6) * np.sin(q4)) / 1000 + (111 * np.cos(q3) * np.cos(q4) * np.cos(q6) * np.sin(q2)) / 1000 + (
                 111 * np.cos(q2) * np.cos(q4) * np.cos(q5) * np.sin(q6)) / 1000 + (
                 111 * np.cos(q3) * np.cos(q5) * np.sin(q2) * np.sin(q4) * np.sin(q6)) / 1000, (111 * np.sin(q6) * (
                np.cos(q5) * np.sin(q2) * np.sin(q3) - np.cos(q2) * np.sin(q4) * np.sin(q5) + np.cos(q3) * np.cos(q4) * np.sin(q2) * np.sin(
            q5))) / 1000,
     (111 * np.cos(q2) * np.cos(q5) * np.cos(q6) * np.sin(q4)) / 1000 - (111 * np.cos(q2) * np.cos(q4) * np.sin(q6)) / 1000 - (
                 111 * np.cos(q3) * np.sin(q2) * np.sin(q4) * np.sin(q6)) / 1000 + (
                 111 * np.cos(q6) * np.sin(q2) * np.sin(q3) * np.sin(q5)) / 1000 - (
                 111 * np.cos(q3) * np.cos(q4) * np.cos(q5) * np.cos(q6) * np.sin(q2)) / 1000, 0]])

    Jw = np.array([[0, -np.sin(q1), np.cos(q1) * np.sin(q2), np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3),
     np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2),
     np.sin(q5) * (np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) - np.cos(q5) * (
                 np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)),
     np.cos(q6) * (np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2)) - np.sin(q6) * (
                 np.cos(q5) * (
                     np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
             q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))],
    [0, np.cos(q1), np.sin(q1) * np.sin(q2), np.cos(q2) * np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q3),
     np.cos(q4) * np.sin(q1) * np.sin(q2) - np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)),
     np.cos(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)) - np.sin(q5) * (
                 np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)), np.sin(q6) * (
                 np.cos(q5) * (np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
             q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3))) - np.cos(q6) * (
                 np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))],
    [1, 0, np.cos(q2), -np.sin(q2) * np.sin(q3), np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4),
     np.cos(q5) * np.sin(q2) * np.sin(q3) - np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)),
     np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5)) + np.cos(q6) * (
                 np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))]])

    J = np.vstack((Jv, Jw))
    return J, Jv, Jw


def eval_symbolic_T_KUKALBR(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]
    q5 = q[4]
    q6 = q[5]
    q7 = q[6]

    T = np.array([[np.sin(q7) * (np.sin(q5) * (
                np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) - np.cos(q5) * (
                            np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3))) - np.cos(q7) * (np.sin(q6) * (
                np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2)) + np.cos(q6) * (
                                                                                                       np.cos(q5) * (
                                                                                                           np.cos(q4) * (
                                                                                                               np.sin(q1) * np.sin(
                                                                                                           q3) - np.cos(
                                                                                                           q1) * np.cos(
                                                                                                           q2) * np.cos(
                                                                                                           q3)) - np.cos(
                                                                                                       q1) * np.sin(
                                                                                                       q2) * np.sin(
                                                                                                       q4)) + np.sin(
                                                                                                   q5) * (np.cos(q3) * np.sin(
                                                                                                   q1) + np.cos(q1) * np.cos(
                                                                                                   q2) * np.sin(q3)))),
     np.sin(q7) * (np.sin(q6) * (
                 np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2)) + np.cos(
         q6) * (np.cos(q5) * (
                 np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
         q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))) + np.cos(q7) * (np.sin(q5) * (
                 np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) - np.cos(
         q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3))),
     np.cos(q6) * (np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(q2)) - np.sin(q6) * (
                 np.cos(q5) * (
                     np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
             q5) * (np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3))), (21 * np.cos(q1) * np.sin(q2)) / 50 + (
                 111 * np.cos(q6) * (np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) + np.cos(q1) * np.cos(q4) * np.sin(
             q2))) / 1000 - (111 * np.sin(q6) * (np.cos(q5) * (
                np.cos(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3)) - np.cos(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (
                                                          np.cos(q3) * np.sin(q1) + np.cos(q1) * np.cos(q2) * np.sin(q3)))) / 1000 + (
                 2 * np.sin(q4) * (np.sin(q1) * np.sin(q3) - np.cos(q1) * np.cos(q2) * np.cos(q3))) / 5 + (
                 2 * np.cos(q1) * np.cos(q4) * np.sin(q2)) / 5],
    [np.cos(q7) * (np.sin(q6) * (
                np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2)) + np.cos(q6) * (
                            np.cos(q5) * (
                                np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(
                            q4)) + np.sin(q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)))) - np.sin(q7) * (
                 np.sin(q5) * (
                     np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) - np.cos(
             q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3))), - np.sin(q7) * (np.sin(q6) * (
                np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2)) + np.cos(q6) * (
                                                                                                np.cos(q5) * (np.cos(q4) * (
                                                                                                    np.cos(q1) * np.sin(
                                                                                                q3) + np.cos(q2) * np.cos(
                                                                                                q3) * np.sin(q1)) + np.sin(
                                                                                            q1) * np.sin(q2) * np.sin(
                                                                                            q4)) + np.sin(q5) * (
                                                                                                            np.cos(q1) * np.cos(
                                                                                                        q3) - np.cos(
                                                                                                        q2) * np.sin(
                                                                                                        q1) * np.sin(
                                                                                                        q3)))) - np.cos(
        q7) * (np.sin(q5) * (
                np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) - np.cos(q5) * (
                           np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3))), np.sin(q6) * (np.cos(q5) * (
                np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(q5) * (
                                                                                                     np.cos(q1) * np.cos(
                                                                                                 q3) - np.cos(q2) * np.sin(
                                                                                                 q1) * np.sin(q3))) - np.cos(
        q6) * (np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2)),
     (21 * np.sin(q1) * np.sin(q2)) / 50 - (111 * np.cos(q6) * (
                 np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) - np.cos(q4) * np.sin(q1) * np.sin(q2))) / 1000 + (
                 111 * np.sin(q6) * (np.cos(q5) * (
                     np.cos(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1)) + np.sin(q1) * np.sin(q2) * np.sin(q4)) + np.sin(
             q5) * (np.cos(q1) * np.cos(q3) - np.cos(q2) * np.sin(q1) * np.sin(q3)))) / 1000 - (
                 2 * np.sin(q4) * (np.cos(q1) * np.sin(q3) + np.cos(q2) * np.cos(q3) * np.sin(q1))) / 5 + (
                 2 * np.cos(q4) * np.sin(q1) * np.sin(q2)) / 5],
    [np.cos(q7) * (np.cos(q6) * (
                np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5)) - np.sin(q6) * (
                            np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) - np.sin(q7) * (
                 np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) - np.cos(q5) * np.sin(q2) * np.sin(q3)),
     - np.cos(q7) * (np.sin(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) - np.cos(q5) * np.sin(q2) * np.sin(q3)) - np.sin(
         q7) * (np.cos(q6) * (
                 np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5)) - np.sin(
         q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))),
     np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(q5)) + np.cos(q6) * (
                 np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4)), (21 * np.cos(q2)) / 50 + (2 * np.cos(q2) * np.cos(q4)) / 5 + (
                 111 * np.sin(q6) * (np.cos(q5) * (np.cos(q2) * np.sin(q4) - np.cos(q3) * np.cos(q4) * np.sin(q2)) + np.sin(q2) * np.sin(q3) * np.sin(
             q5))) / 1000 + (111 * np.cos(q6) * (np.cos(q2) * np.cos(q4) + np.cos(q3) * np.sin(q2) * np.sin(q4))) / 1000 + (
                 2 * np.cos(q3) * np.sin(q2) * np.sin(q4)) / 5 + 9 / 25],
                [0, 0, 0, 1]])
    return T



